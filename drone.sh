log_type=log_pid
cd ~/git/simplest_drone
cargo run --release --features $log_type
cd ~/git/drone_plotter
cargo build --release --features $log_type
sleep 5
socat file:/dev/tty.usbmodem83101,ispeed=115200,ospeed=115200,raw,echo=0,nonblock,waitlock=/tmp/usbmodem.lock - | ~/git/drone_plotter/target/release/stream_plotter