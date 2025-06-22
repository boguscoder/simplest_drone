cd ~/git/simplest_drone
cargo run --release --features telemetry
cd ~/git/drone_plotter
cargo build --release
sleep 3
~/git/drone_plotter/target/release/stream_plotter