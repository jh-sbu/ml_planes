//! Compute-backend selector shared by the `train_ppo` / `train_bc` binaries.
//!
//! `NdArray` (CPU) is the default: every documented training workflow (the
//! `train-evaluate-improve` / `train-evaluate-optimize` skills, `evaluate_policy`)
//! already runs on it, and it needs no GPU. `Wgpu` is available only when the
//! crate is built with the opt-in `wgpu` cargo feature (`training = [...]` no
//! longer pulls in `burn/wgpu`; `wgpu = ["training", "burn/wgpu"]` does), so a
//! plain `--features training` build never compiles the wgpu/cubecl stack.

/// Which `burn` backend a training binary should instantiate `Autodiff<_>` over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Backend {
    NdArray,
    #[cfg(feature = "wgpu")]
    Wgpu,
}

impl Default for Backend {
    fn default() -> Self {
        Self::NdArray
    }
}

impl Backend {
    /// Parse a `--backend` CLI value. `"wgpu"` is recognised even when the
    /// `wgpu` feature is off, so the error can point at the fix (rebuild with
    /// `--features wgpu`) rather than reading like a typo.
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "ndarray" | "cpu" => Ok(Self::NdArray),
            #[cfg(feature = "wgpu")]
            "wgpu" => Ok(Self::Wgpu),
            #[cfg(not(feature = "wgpu"))]
            "wgpu" => Err(
                "--backend wgpu requires rebuilding with --features \"training wgpu\" \
                 (the default `training` build no longer includes the wgpu/cubecl stack)"
                    .to_string(),
            ),
            other => Err(format!(
                "Unsupported --backend '{other}'. Use 'ndarray' or 'cpu'{}.",
                if cfg!(feature = "wgpu") {
                    ", or 'wgpu'"
                } else {
                    ""
                }
            )),
        }
    }

    /// Human-readable label, printed at startup so a run is self-documenting.
    pub fn label(&self) -> &'static str {
        match self {
            Self::NdArray => "ndarray",
            #[cfg(feature = "wgpu")]
            Self::Wgpu => "wgpu",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_backend_is_ndarray() {
        assert_eq!(Backend::default(), Backend::NdArray);
        assert_eq!(Backend::default().label(), "ndarray");
    }

    #[test]
    fn parse_accepts_cpu_aliases() {
        assert_eq!(Backend::parse("ndarray"), Ok(Backend::NdArray));
        assert_eq!(Backend::parse("cpu"), Ok(Backend::NdArray));
    }

    #[test]
    fn parse_rejects_unknown_backend() {
        let err = Backend::parse("potato").unwrap_err();
        assert!(
            err.contains("potato"),
            "error should name the bad value: {err}"
        );
    }

    #[cfg(not(feature = "wgpu"))]
    #[test]
    fn wgpu_requires_the_wgpu_feature() {
        let err = Backend::parse("wgpu").unwrap_err();
        assert!(
            err.contains("wgpu"),
            "error should mention the wgpu feature: {err}"
        );
    }

    #[cfg(feature = "wgpu")]
    #[test]
    fn parse_accepts_wgpu_when_enabled() {
        assert_eq!(Backend::parse("wgpu"), Ok(Backend::Wgpu));
    }
}
