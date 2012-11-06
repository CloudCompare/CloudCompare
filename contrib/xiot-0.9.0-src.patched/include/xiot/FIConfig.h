/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/
#ifndef FI_FICONFIG_H
#define FI_FICONFIG_H


#if defined(_WIN32)
# if defined(openFI_EXPORTS)
#  define OPENFI_EXPORT __declspec(dllexport)
# else
#  define OPENFI_EXPORT __declspec(dllimport)
# endif
#else
# define OPENFI_EXPORT
#endif

#if defined(_MSC_VER) 
# pragma warning (disable: 4275) /* non-DLL-interface base class used */
# pragma warning (disable: 4251) /* needs to have dll-interface to be used by clients */
/* No warning for safe windows only functions */
# define _CRT_SECURE_NO_WARNINGS
#endif


#endif // FI_FICONFIG_H
