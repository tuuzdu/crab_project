/*
 * textfile.h
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-2010 Filipe Coelho <falktx@gmail.com>
 *
 * QtSixA can be redistributed and/or modified under the terms of the GNU General
 * Public License (Version 2), as published by the Free Software Foundation.
 * A copy of the license is included in the QtSixA source code, or can be found
 * online at www.gnu.org/licenses.
 *
 * QtSixA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 */

#ifndef TEXTFILE_H
#define TEXTFILE_H

#include <cstdlib>

inline char *find_key(char *map, size_t size, const char *key, size_t len, int icase);
char *read_key(const char *pathname, const char *key, int icase);
int textfile_get_int(const char *pathname, const char *key, int default_key);

#endif // TEXTFILE_H
