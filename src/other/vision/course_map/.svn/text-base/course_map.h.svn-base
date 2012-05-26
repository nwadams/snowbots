#ifndef course_map_h_included
#define course_map_h_included

#include <vector>

using namespace std;

/** Represents a section of a course (eg, a turn, a straightaway). **/
typedef struct {
	double length;
	int turn;
} CourseSection;

/** Loads the course contained in mapfile into the vector map.
 * @return true if successful, false otherwise.
**/
bool loadMap(const char *mapfile, vector<CourseSection> &map);

#endif
