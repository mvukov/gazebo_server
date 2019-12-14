// Copyright 2019 Milan Vukov. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "gazebo_server/helpers.h"

#include <gazebo/common/Console.hh>
#include <sdf/parser_urdf.hh>
#include <tinyxml.h>

namespace gazebo_server {

std::string UrdfToSdf(const std::string& model_urdf_xml) {
  TiXmlDocument model_sdf_doc;
  std::string model_sdf_xml;
  try {
    TiXmlDocument model_urdf_doc;
    model_urdf_doc.Parse(model_urdf_xml.c_str());

    sdf::URDF2SDF urdf2sdf;
    model_sdf_doc = urdf2sdf.InitModelDoc(&model_urdf_doc);
  } catch (const std::exception& ex) {
    gzerr << "Failed to convert URDF to SDF! Reason: " << ex.what()
          << std::endl;
    return model_sdf_xml;
  }

  TiXmlPrinter xml_printer;
  model_sdf_doc.Accept(&xml_printer);
  model_sdf_xml = xml_printer.Str();
  return model_sdf_xml;
}

std::string GetRobotName(const std::string& model_sdf_xml) {
  TiXmlDocument model_doc;
  model_doc.Parse(model_sdf_xml.c_str());
  TiXmlElement* sdf_element = model_doc.FirstChildElement("sdf");
  if (!sdf_element) {
    gzerr << "Got an invalid SDF XML (no sdf tag)!" << std::endl;
    return std::string();
  }
  TiXmlElement* model_element = sdf_element->FirstChildElement("model");
  if (!model_element || !model_element->Attribute("name")) {
    gzerr << "Got an invalid SDF XML (no model tag and/or name)!" << std::endl;
    return std::string();
  }
  return std::string(model_element->Attribute("name"));
}

}  // namespace gazebo_server
