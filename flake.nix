{
  description = "Rust development environment with Cargo dependencies";

  # Specify the inputs, such as nixpkgs
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  # Define the outputs
  outputs = { self, nixpkgs, flake-utils }: flake-utils.lib.eachDefaultSystem (system: let
    pkgs = import nixpkgs { inherit system; };
  in {
    # Define a devShell for development
    devShell = pkgs.mkShell {
      # Add Rust and Cargo to the environment
      buildInputs = [
        pkgs.rustup
      ];

      # Optionally, set environment variables
      CARGO_HOME = "./.cargo";
      RUST_BACKTRACE = "1"; # Enable backtrace for debugging

      # Optional shellHook to fetch dependencies when entering the shell
      shellHook = ''
        echo "Entering Rust development environment..."
        rustup default nightly
        cargo install rerun-cli
        cargo fetch # Pre-fetch dependencies defined in Cargo.toml
      '';
    };
  });
}

