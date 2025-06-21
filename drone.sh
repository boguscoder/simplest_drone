log_type=log_att
cd ~/git/simplest_drone
cargo run --release --features telemetry
cd ~/git/drone_plotter
cargo build --release
sleep 3
socat file:/dev/tty.usbmodem83101,ispeed=115200,ospeed=115200,raw,echo=0,nonblock,waitlock=/tmp/usbmodem.lock EXEC:'$HOME/git/drone_plotter/target/release/stream_plotter'