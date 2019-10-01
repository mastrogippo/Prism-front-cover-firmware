// Prism front cover
// Copyright (C) 2019 Mastro Gippo - Michele Brunelli
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef PRISMDEBUG_HPP
#define PRISMDEBUG_HPP
/*
#ifdef PRISM_DEBUG
#define DEBUG_BEGIN(x) Serial.begin(x)
#define DEBUG(x) Serial.print(x)
#define DEBUGLN(x) Serial.println(x)
#else*/
#define DEBUG_BEGIN(x)
#define DEBUG(x)
#define DEBUGLN(x)
//#endif

#endif