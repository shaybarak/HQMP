#include "mrmpapplication.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MRMPApplication w;
	a.setActiveWindow(&w);
	w.show();
	/*a.setQuitOnLastWindowClosed(true);
	a.connect( qApp, SIGNAL( lastWindowClosed() ), qApp, SLOT( quit() ) );*/

	return a.exec();
}
