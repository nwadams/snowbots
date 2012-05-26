#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_SIZE 100

FILE *safe_open(char *filename, char *mode) {
	FILE *file = fopen(filename, mode);
	char error_msg[LINE_SIZE];
	if ( file == NULL ) {
		sprintf(error_msg,
		        "safe_open(): unable to open file '%s'",
		        filename /*,
		        strerror()*/ );
		perror(error_msg);
		exit(1);
	}
	return file;
}

void read_conf_file() {
	FILE *conf_file;
	char line[LINE_SIZE];

	conf_file = safe_open("conf/robot.conf", "r");

	while ( fgets(line, LINE_SIZE, conf_file) != NULL ) {
		char *comment_begins;
		char setting[LINE_SIZE];
		char value[LINE_SIZE];

		// If there is a comment
		comment_begins = strchr(line, '#');
		if ( comment_begins != NULL ) {
			// cut the string off at that point.
			*comment_begins = '\0';
		}

		// Parse the line into setting and value.
		if ( sscanf(line, "%s = %s", setting, value) != EOF ) {
			printf("setting '%s' is %s\n", setting, value);
		}
		
		// TODO: based on value of "setting", convert "value" into
		// approprate type and store as global variable or robot config
		// struct.
	}
}

int main() {
	read_conf_file();
	return 0;
}


