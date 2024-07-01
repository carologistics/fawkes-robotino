/***************************************************************************
 *  base64.h - simple base64 library by Rene Nyffenegger
 *
 *  Created: Sun Jun 30 17:36:00 2024
 *  Copyright  2022 Rene Nyffenegger
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A
#define BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A

#include <string>

#if __cplusplus >= 201703L
#	include <string_view>
#endif // __cplusplus >= 201703L

std::string base64_encode(std::string const &s, bool url = false);
std::string base64_encode_pem(std::string const &s);
std::string base64_encode_mime(std::string const &s);

std::string base64_decode(std::string const &s, bool remove_linebreaks = false);
std::string base64_encode(unsigned char const *, size_t len, bool url = false);

#if __cplusplus >= 201703L
//
// Interface with std::string_view rather than const std::string&
// Requires C++17
// Provided by Yannic Bonenberger (https://github.com/Yannic)
//
std::string base64_encode(std::string_view s, bool url = false);
std::string base64_encode_pem(std::string_view s);
std::string base64_encode_mime(std::string_view s);

std::string base64_decode(std::string_view s, bool remove_linebreaks = false);
#endif // __cplusplus >= 201703L

#endif /* BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A */
