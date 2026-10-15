use std::{env, fs, path::Path};

fn main() {
    let root = env::var("CARGO_MANIFEST_DIR").unwrap();
    let root = Path::new(&root);
    let out = env::var("OUT_DIR").unwrap();
    let out = Path::new(&out);

    // Only replace the build information if it changed, to keep the file's timestamp.
    let new = out.join("built.rs.new");
    built::write_built_file_with_opts(Some(root), &new)
        .expect("Failed to acquire build-time information");
    let built = out.join("built.rs");
    if fs::read(&new).ok() != fs::read(&built).ok() {
        fs::rename(&new, &built).unwrap();
    }

    println!("cargo:rerun-if-changed=memory.x");
    rerun_if_version_changed(root);
}

/// Rerun this script when the git version or the dirty flag in the build information may
/// change. Cargo otherwise only reruns it when one of the paths named here changes, and `built`
/// names none.
///
/// A rerun recompiles the whole package. To avoid that on every edit, the sources are only
/// watched while the tree is clean: further edits cannot change the dirty flag. Reverting all
/// edits by hand leaves the flag set until the next change to the index or HEAD.
fn rerun_if_version_changed(root: &Path) {
    let Ok(repo) = git2::Repository::discover(root) else {
        return;
    };

    // Commits, checkouts, tags (`git describe`) and staged changes. A worktree has its own
    // HEAD and index; the refs are shared.
    let common = repo.commondir();
    let mut paths = vec![
        repo.path().join("HEAD"),
        repo.path().join("index"),
        common.join("packed-refs"),
        common.join("refs/tags"),
    ];
    if let Ok(head) = repo.head()
        && head.is_branch()
        && let Some(name) = head.name()
    {
        paths.push(common.join(name));
    }

    // Edits to the sources of this crate and of the workspace members, as long as the tree is
    // clean (with the same criteria as `built`).
    let mut options = git2::StatusOptions::new();
    options.include_untracked(false).include_ignored(false);
    let clean = repo.statuses(Some(&mut options)).is_ok_and(|statuses| {
        statuses
            .iter()
            .all(|status| status.status() == git2::Status::CURRENT)
    });
    if clean {
        paths.extend([
            root.join("Cargo.toml"),
            root.join("Cargo.lock"),
            root.join("src"),
        ]);
        for entry in fs::read_dir(root).unwrap().flatten() {
            let member = entry.path();
            if member.join("Cargo.toml").is_file() {
                paths.extend([member.join("Cargo.toml"), member.join("src")]);
            }
        }
    }

    // Cargo always reruns the script for paths that do not exist.
    for path in paths.iter().filter(|path| path.exists()) {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}
