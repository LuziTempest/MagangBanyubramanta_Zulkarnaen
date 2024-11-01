#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

bool isKeyPressed(int key) {
    struct termios oldt, newt;
    int oldf;
    char ch;
    bool keyPressed = false;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    if (ch == key) {
        keyPressed = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return keyPressed;
}

class KeyPressBehavior {
public:
    KeyPressBehavior(int cek_key){
        key = cek_key;
    }

    void execute() {
        if (isKeyPressed(key)) {
            cout << "Success: Key is pressed.\n";
        } else {
            cout << "Failure: Key is not pressed.\n";
        }
    }
private:
    int key;
};

int main() {
    int keyToCheck = 32;
    KeyPressBehavior keyPressBehavior(keyToCheck);

    cout << "Press the backspace key within 3 seconds...\n";
    sleep(3); // Jeda 3 detik buat nekan tombol
    
    keyPressBehavior.execute();

    return 0;
}