/// A macro similar to `log::info!`, but with built-in counter-based rate limiting.
///
/// This macro only prints the log message if `count_limit` calls have occurred
/// since the last time this *specific call site* printed a message.
///
/// Each call to `rl_log!` will have its own independent counter.
///
/// # Usage:
/// `rl_log!(100, "This message will print every 100 calls from here");`
/// `rl_log!(50, "Value: {}", my_variable);`
///
/// # Arguments:
/// - `$count_limit`: The minimum number of calls (iterations) between consecutive logs
///   from this specific call site. Must be a `usize` literal or constant.
/// - `$($arg:tt)*`: The format string and arguments, identical to `log::info!`.
#[macro_export]
macro_rules! rl_log {
    ($count_limit:expr, $($arg:tt)*) => {
        static mut __CALL_COUNTER: usize = 0;

        let limit = $count_limit as usize;
        let effective_limit = if limit == 0 { 1 } else { limit };

        let pass_log = unsafe {
            __CALL_COUNTER += 1;
            __CALL_COUNTER % effective_limit == 0
        };
        if pass_log {
            log::info!($($arg)*);
        }
    };
}
