/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, CITEC, Bielefeld University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Robert Haschke */

#ifndef URDF_PARSER_UTIL_H
#define URDF_PARSER_UTIL_H

#include <urdf_exception/exception.h>
#include <urdf_model/utils.h>

namespace urdf {

/* attribute parsing is accomplished by boost::lexical_cast by default */
template <typename T>
T parseAttribute(const char* value);

template<>
inline std::string parseAttribute<std::string>(const char* value)
{
  return value;
}

template<>
inline double parseAttribute<double>(const char* value)
{
  return strToDouble(value);
}

template<>
inline unsigned int parseAttribute<unsigned int>(const char* value)
{
  return std::stoul(value);
}

/** Check existence of an attribute called attr and parse its values as T.
 *  If there is no such attribute, return *default_value if not NULL.
 *  Otherwise throw a ParseError.
 */
template <typename T>
T parseAttribute(const TiXmlElement &tag, const char* attr, const T* default_value=NULL)
{
  const char* value = tag.Attribute(attr);
  if (!value)
  {
    if (default_value) return *default_value;
    else throw ParseError(std::string("missing '") + attr + "'' attribute");
  }

  try
  {
    return parseAttribute<T>(value);
  }
  catch (const std::exception &e)
  {
    throw ParseError(std::string("failed to parse '") + attr + "' attribute: " + e.what());
  }
}

}

#endif
