/*************************************************************
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
* 
*************************************************************/
#ifndef MENU_MGR_H
#define MENU_MGR_H

#include <vector>
#include <map>
#include <string>

class MenuItem
{
	protected:
		typedef enum {CFunction, CFunctionWithContext, CPPFunctor, CPPMember} MenuItemType;
		MenuItemType type;
		std::string helptext;
	public:
		MenuItem(const std::string & helpstring,MenuItemType tp) : type(tp), helptext(helpstring) {}
		virtual ~MenuItem() {}
		virtual int operator()(const std::vector<std::string> & args) = 0;
		virtual const std::string & helpstring() const {return helptext;}
};

class MenuItemCFunction : public MenuItem
{
	public:
		typedef int (*MenuCFunction)(const std::vector<std::string> & args);
		typedef int (*MenuCFunctionWithContext)(void * context, const std::vector<std::string> & args);
	protected:
		MenuCFunction cfunction;
		MenuCFunctionWithContext cfuncctxt;
		void * context;
	public:
		MenuItemCFunction(MenuCFunction f, const std::string & helpstring) : 
			MenuItem(helpstring, CFunction), cfunction(f), cfuncctxt(NULL), context(NULL) {}
		MenuItemCFunction(void * arg, MenuCFunctionWithContext f, const std::string & helpstring) : 
			MenuItem(helpstring, CFunctionWithContext), cfunction(NULL), cfuncctxt(f), context(arg) {}
		virtual int operator()(const std::vector<std::string> & args) {
			switch (type) {
				case CFunction:
					return cfunction(args);
				case CFunctionWithContext:
					return cfuncctxt(context,args);
				default:
					return -1;
			}
		}
};

template <class C>
class MenuItemConstFunctor : public MenuItem
{
	protected:
		const C & functor;
	public:
		MenuItemConstFunctor(const C & obj, const std::string & helpstring) :
			MenuItem(helpstring, CPPFunctor), functor(obj) {}
		virtual int operator()(const std::vector<std::string> & args) {
			return functor(args);
		}
};

template <class C>
class MenuItemFunctor : public MenuItem
{
	protected:
		C & functor;
	public:
		MenuItemFunctor(C & obj, const std::string & helpstring) :
			MenuItem(helpstring, CPPFunctor), functor(obj) {}
		virtual int operator()(const std::vector<std::string> & args) {
			return functor(args);
		}
};

template <class C>
class MenuItemConstMember : public MenuItem
{
	public:
		typedef int (C::*MemberMethod)(const std::vector<std::string> & args) const;
	protected:
		const C * object;
		MemberMethod method;
	public:
		MenuItemConstMember(const C * obj,
				MemberMethod f, const std::string & helpstring) :
			MenuItem(helpstring, CPPMember), object(obj), method(f) {}
		virtual int operator()(const std::vector<std::string> & args) {
			return (object->*method)(args);
		}
};

template <class C>
class MenuItemMember : public MenuItem
{
	public:
		typedef int (C::*MemberMethod)(const std::vector<std::string> & args) ;
	protected:
		C * object;
		MemberMethod method;
	public:
		MenuItemMember(C * obj,
				MemberMethod f, const std::string & helpstring) :
			MenuItem(helpstring, CPPMember), object(obj), method(f) {}
		virtual int operator()(const std::vector<std::string> & args) {
			return (object->*method)(args);
		}
};

class MenuManager
{
	protected:
		bool endoffile;
		std::string prompt;
		typedef std::map<std::string,MenuItem*,std::less<std::string> > MenuDefinition;
		MenuDefinition menu;
	public:
		// Add help object
		MenuManager(const std::string &_prompt="#>");
		~MenuManager() {clearAll();}

		void clearAll();
		void add(const std::string & name, MenuItem *item);

		// Argument, so that this can be called as a menuitem
		int printHelp(const std::vector<std::string> & args) const;
		int runOnce();

		bool endOfFile() const {
			return endoffile;
		}


		struct ArgumentException : public std::exception {
			std::string text;
			ArgumentException(const std::string & s) : text(s) {}
			virtual ~ArgumentException() throw () {}
			const char *what() const throw () { return text.c_str(); }
		};
#ifdef WIN32
		static int checkArguments(const std::vector<std::string> & args,
				unsigned int minArgs, unsigned int maxArgs);
#else
		static int checkArguments(const std::vector<std::string> & args,
				unsigned int minArgs, unsigned int maxArgs) throw (ArgumentException);
#endif

};



#endif // MENU_MGR_H
