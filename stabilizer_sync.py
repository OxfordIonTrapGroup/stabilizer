import json
import asyncio
from collections import OrderedDict as OD
import logging

import numpy as np
import serial

logger = logging.getLogger()


class StabilizerError(Exception):
    pass


class StabilizerConfig:
    def connect(self, host, port=1235):
        self.dev = serial.serial_for_url("socket://"+host+":"+str(port))

    def set(self, channel, iir, dac):
        up = OD([("channel", channel), ("iir", iir.as_dict()), ("cpu_dac", dac.as_dict())])
        s = json.dumps(up, separators=(",", ":"))
        assert "\n" not in s
        logger.debug("send %s", s)
        self.dev.write(s.encode() + b"\n")
        r = (self.dev.readline()).decode()
        logger.debug("recv %s", r)
        ret = json.loads(r, object_pairs_hook=OD)
        if ret["code"] != 200:
            raise StabilizerError(ret)
        return ret


class IIR:
    t_update = 2e-6
    full_scale = float((1 << 15) - 1)

    def __init__(self):
        self.ba = np.zeros(5, np.float32)
        self.y_offset = 0.
        self.y_min = -self.full_scale - 1
        self.y_max = self.full_scale

    def as_dict(self):
        iir = OD()
        iir["ba"] = [float(_) for _ in self.ba]
        iir["y_offset"] = self.y_offset
        iir["y_min"] = self.y_min
        iir["y_max"] = self.y_max
        return iir

    def configure_pi(self, kp, ki, g=0.):
        ki = np.copysign(ki, kp)*self.t_update*2
        g = np.copysign(g, kp)
        eps = np.finfo(np.float32).eps
        if abs(ki) < eps:
            a1, b0, b1 = 0., kp, 0.
        else:
            if abs(g) < eps:
                c = 1.
            else:
                c = 1./(1. + ki/g)
            a1 = 2*c - 1.
            b0 = ki*c + kp
            b1 = ki*c - a1*kp
            if abs(b0 + b1) < eps:
                raise ValueError("low integrator gain and/or gain limit")
        self.ba[0] = b0
        self.ba[1] = b1
        self.ba[2] = 0.
        self.ba[3] = a1
        self.ba[4] = 0.

    def set_x_offset(self, o):
        b = self.ba[:3].sum()*self.full_scale
        self.y_offset = b*o


class CPU_DAC:
    full_scale = 0xfff

    def __init__(self):
        self.en = True
        self.out = np.zeros(1, np.float32)

    def set_out(self, out):
        assert out >= 0 and out <= 0xfff, "cpu dac setting out of range"
        self.out = out

    def set_out_scaled(self, out):
        assert out >= 0. and out <= 1.0, "cpu dac setting out of range"
        self.out = int(out*0xfff)

    def set_en(self, en):
        self.en = en

    def as_dict(self):
        dac = OD()
        dac["out"] = int(self.out)
        dac["en"] = bool(self.en)
        return dac


if __name__ == "__main__":
    import argparse

    def str2bool(v):
        if isinstance(v, bool):
           return v
        if v.lower() in ('yes', 'true', 't', 'y', '1'):
            return True
        elif v.lower() in ('no', 'false', 'f', 'n', '0'):
            return False
        else:
            raise argparse.ArgumentTypeError('Boolean value expected.')


    p = argparse.ArgumentParser()
    p.add_argument("-s", "--stabilizer", default="10.255.6.169")
    p.add_argument("-c", "--channel", default=0, type=int,
                   help="Stabilizer channel to configure")
    p.add_argument("-o", "--offset", default=0., type=float,
                   help="input offset, in units of full scale")
    p.add_argument("-p", "--proportional-gain", default=1., type=float,
                   help="Proportional gain, in units of 1")
    p.add_argument("-i", "--integral-gain", default=0., type=float,
                   help="Integral gain, in units of Hz, "
                        "sign taken from proportional-gain")
    p.add_argument("-e", "--cpu-dac-en", default=True, type=str2bool,
                   help="CPU-DAC enable, 0 for off")
    p.add_argument("-d", "--cpu-dac-out", default=0, type=int,
                   help="CPU-DAC output, as u12 from GND to 2.04 V ")

    args = p.parse_args()

    loop = asyncio.get_event_loop()
    # loop.set_debug(True)
    logging.basicConfig(level=logging.DEBUG)

    def main():
        d = CPU_DAC()
        d.set_out(args.cpu_dac_out)
        d.set_en(args.cpu_dac_en)
        i = IIR()
        i.configure_pi(args.proportional_gain, args.integral_gain)
        i.set_x_offset(args.offset)
        s = StabilizerConfig()
        s.connect(args.stabilizer)
        assert args.channel in range(2)
        r = s.set(args.channel, i, d)


    main()

