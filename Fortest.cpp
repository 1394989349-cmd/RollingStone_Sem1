#define WINVER 0x0500
#define _WIN32_WINNT 0x0500
#include <graphics.h>
#include <conio.h>
signed main()
{
	initgraph(640, 480);
	setbkcolor(WHITE);
	cleardevice();
	setcolor(RED);
	rectangle(100, 100, 300, 300);
	_getch();
	closegraph();
	return 0;
}