#include "motorized_shoe/keyboard_input.hpp"

#include <iostream>
#include <poll.h>
#include <unistd.h>

namespace motorized_shoe {

KeyboardInput::KeyboardInput() {
    is_tty_ = isatty(STDIN_FILENO) != 0;
}

KeyboardInput::~KeyboardInput() {
    stop();
}

void KeyboardInput::start(std::function<void(char)> on_key) {
    callback_ = std::move(on_key);

    if (!is_tty_) {
        std::cerr << "[keyboard] stdin is not a TTY; keyboard input disabled\n";
        return;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) == 0) {
        termios_saved_ = true;
        struct termios raw = original_termios_;
        raw.c_lflag &= ~(static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    run_.store(true, std::memory_order_release);
    thread_ = std::thread([this]() {
        while (run_.load(std::memory_order_acquire)) {
            struct pollfd pfd {};
            pfd.fd = STDIN_FILENO;
            pfd.events = POLLIN;
            const int rc = poll(&pfd, 1, 100);  // 100 ms timeout to check run_
            if (rc < 0) {
                continue;
            }
            if (rc == 0) {
                continue;
            }
            char c = 0;
            const ssize_t n = read(STDIN_FILENO, &c, 1);
            if (n == 1 && callback_) {
                callback_(c);
            }
        }
    });
}

void KeyboardInput::stop() {
    run_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (termios_saved_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        termios_saved_ = false;
    }
}

}  // namespace motorized_shoe
