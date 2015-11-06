#include "utils.h"

bool exists(string filename){
	ifstream file(filename);
	return file.good();
}
