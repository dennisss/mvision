#include "utils.h"

#include <fstream>


static char *laststr = NULL;
char *spacetok(char *str){
	char c, *tok;


	if(str == NULL)
		str = laststr;


	// Skip initial whitespace
	c = *str;
	while(c != '\0' && isspace(c)){
		str++;
		c = *str;
	}


	if(c == '\0')
		return NULL;


	tok = str;

	while(c != '\0' && !isspace(c)){
		str++;
		c = *str;
	}

	if(c == '\0'){ // It is the actual end of the string
		laststr = str;
	}
	else{
		*str = '\0';
		laststr = str + 1;
	}


	return tok;
}


// Reads the first non-empty line as tokens and puts them into the vector
istream& operator>>(istream& is, vector<string>& toks){
	string line;
	toks.clear();

	while(!is.eof() && toks.size() == 0){
		getline(is, line);

		char *tok = spacetok((char *) line.c_str() /*cline*/);
		while(tok != NULL){
			toks.push_back(tok);
			tok = spacetok(NULL);
		}
	}


	return is;
}

bool exists(string filename){
	ifstream file(filename);
	return file.good();
}
