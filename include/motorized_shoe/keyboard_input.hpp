#ifndef MOTORIZED_SHOE_KEYBOARD_INPUT_HPP
#define MOTORIZED_SHOE_KEYBOARD_INPUT_HPP

#include <atomic>
#include <functional>
#include <termios.h>
#include <thread>

namespace motorized_shoe {

// Reads single characters from stdin without line buffering and forwards them
// to a callback on a dedicated thread. Restores termios on destruction.
//
// Only active when stdin is a TTY; otherwise start() is a no-op (so piping or
// systemd-launched runs don't break).
class KeyboardInput {
public:
    KeyboardInput();
    ~KeyboardInput();

    KeyboardInput(const KeyboardInput&) = delete;
    KeyboardInput& operator=(const KeyboardInput&) = delete;

    void start(std::function<void(char)> on_key);
    void stop();

private:
    std::function<void(char)> callback_;
    std::thread thread_;
    std::atomic<bool> run_{false};
    bool termios_saved_ = false;
    bool is_tty_ = false;
    struct termios original_termios_ {};
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_KEYBOARD_INPUT_HPP
