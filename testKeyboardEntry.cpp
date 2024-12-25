#include <iostream>
#include <conio.h>  // _kbhit(), _getch()
#include <windows.h>

/*

int main()
{
    int counter = 0;
    char exitflag = '\0';
    while (1)
    {
        std::cout << ++counter << ": hello error!\n";

        Sleep(50);

        //函数名：kbhit()
        //功能及返回值： 检查当前是否有键盘输入，若有则返回一个非0值，否则返回0
        if (_kbhit())
        {
            // 函数名: getch()
            // 功能及返回值: 从键盘上读取到的字符
            exitflag = _getch();
            if (exitflag == 'q' || exitflag == 'Q')
            {
                std::cout << "用户输入:" << exitflag << ", 退出循环\n";
                break;
            }           
        }
    }

    std::cout << "已经退出了循环\n";

    return 0;
}

// 注：Windows下不推荐使用POSIX。建议使用使用标准C++相似的名称：_kbhit(), _getch()


*/