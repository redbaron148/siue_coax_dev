/*********************************************************************
* 
* API and Communication library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
* Send modification or corrections as patches (diff -Naur)
* Copyright: Skybotix AG, 2009-2012
* 
* All rights reserved.
* 
* Skybotix API is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* Skybotix API is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
* 
*********************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#ifdef WIN32
#define sscanf sscanf_s
#define sprintf sprintf_s
#define strcpy strcpy_s
#define strncpy strncpy_s
#endif


#ifdef USE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif

#include "MenuManager.h"


MenuManager::MenuManager(const std::string &_prompt) : 
	prompt(_prompt) 
{
	endoffile = false;
#if 0
	add("help",new MenuItemCFunction(this,__printHelp,"Display this help message"));
	add("?",new MenuItemCFunction(this,__printHelp,"Display this help message"));
#else
	add("help",new MenuItemConstMember<MenuManager>(this,&MenuManager::printHelp,"Display this help message"));
	add("?",new MenuItemConstMember<MenuManager>(this,&MenuManager::printHelp,"Display this help message"));
#endif
	
}

void MenuManager::clearAll()
{
	MenuDefinition::iterator it;
	for (it=menu.begin();it!=menu.end();it++) {
		delete it->second;
	}
	menu.clear();
}

void MenuManager::add(const std::string & name, MenuItem *item)
{
	MenuDefinition::iterator it = menu.find(name);
	if (it != menu.end()) {
		delete it->second;
		menu.erase(it);
	}
	menu.insert(std::pair<const std::string, MenuItem*>(name,item));
}

int MenuManager::printHelp(const std::vector<std::string> & args) const
{
	MenuDefinition::const_iterator it;
	unsigned int length = 0;
	printf("Available commands:\n");
	for (it=menu.begin();it!=menu.end();it++) {
		length = std::max(it->first.size(),(size_t)length);
	}
	char templstr[64];
	sprintf(templstr,"%%%ds :\t%%s\n",length);

	for (it=menu.begin();it!=menu.end();it++) {
		printf(templstr,it->first.c_str(), it->second->helpstring().c_str());
	}
	return 0;
}

int MenuManager::runOnce()
{
	size_t pos;
	std::string line;
#ifdef USE_READLINE
	char *linebuf=NULL;
#else
	char linebuf[1024]="";
#endif

#ifdef USE_READLINE
	linebuf = readline(prompt.c_str());
	if (!linebuf) {
		// Ctrl-D -> exit
		endoffile = true;
		return -1;
	}

	line = linebuf;
	free(linebuf);
	pos = line.find_first_not_of(" \t\n\t");
	if (pos != std::string::npos) {
		line = line.substr(pos,line.size()-pos);
	}
	if (!line.empty()) {
		// If there is still something in the buffer, remember it
		add_history(line.c_str());
	}
#else
	printf(prompt.c_str());fflush(NULL);
	fgets(linebuf,1023,stdin);
	if (linebuf[0] == 0) {
		// Ctrl-D -> exit
		endoffile = true;
		return -1;
	}
	line = linebuf;
	pos = line.find_first_not_of(" \t\n\t");
	line = line.substr(pos,line.size()-pos);
#endif
	if (line.empty()) return 0;
	// printf("Line: --%s--\n",line.c_str());

	std::vector<std::string> args;
	while (!line.empty()) {
		std::string arg = line;
		pos = arg.find_first_of(" \t\n\t");
		arg = arg.substr(0,pos);
		// printf("Arg: --%s--\n",arg.c_str());
		if (pos != std::string::npos) {
			line = line.substr(pos,std::string::npos);
		} else {
			line.clear();
		}
		args.push_back(arg);
		pos = line.find_first_not_of(" \t\n\t");
		if (pos != std::string::npos) {
			line = line.substr(pos,std::string::npos);
		} else {
			line.clear();
		}
		// printf("Line: --%s--\n",line.c_str());
	}
	
	MenuDefinition::iterator it = menu.find(args[0]);
	int res = -1;
	if (it != menu.end()) {
		try {
			res =  (*(it->second))(args);
			printf("Result: %d\n",res);
		} catch (const ArgumentException & e) {
			printf("Argument error '%s'\n",e.what());
			res = -1;
		}
	} else {
		printf("Command '%s' not recognised\n",args[0].c_str());
	}
	return res;
}

#ifdef WIN32
int MenuManager::checkArguments(const std::vector<std::string> & args,
		unsigned int minArgs, unsigned int maxArgs) 
#else
int MenuManager::checkArguments(const std::vector<std::string> & args,
		unsigned int minArgs, unsigned int maxArgs) throw (ArgumentException)
#endif
{
	assert(args.size()>=1);
	if ((args.size()-1) < minArgs) throw ArgumentException("Not enough argument");
	if ((args.size()-1) > maxArgs) throw ArgumentException("Not enough argument");
	return 0;
}


