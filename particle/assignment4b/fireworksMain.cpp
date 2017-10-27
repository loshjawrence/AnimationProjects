#include "FireworksViewer.h"
#include <ctime>

int main(int argc, char** argv)
{
	srand(time(NULL));
    FireworksViewer viewer;
	viewer.init(argc, argv);
	viewer.run();

	return 0;
}

