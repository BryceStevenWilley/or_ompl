/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <openrave/plugin.h>

#include <or_ompl/OMPLPlanner.h>
#include <or_ompl/OMPLConversions.h>
#include <or_ompl/OMPLSimplifer.h>
#include <or_ompl/OMPLOptSimplifier.h>
#include <or_ompl/OMPLNothing.h>
#include <or_ompl/PlannerRegistry.h>
#include <or_ompl/SimplifierRegistry.h>

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream &sinput, EnvironmentBasePtr penv) {

    std::vector<std::string> const planner_names
        = or_ompl::registry::getPlannerNames();

    std::vector<std::string> const simplifier_names = or_ompl::registry::getSimplifierNames();

    if (type == PT_Planner && boost::starts_with(interfacename, "ompl_")) {
        // Handle OMPLSimplifier as a special case. This doesn't implement the
        // planning interface, so we can't auto-generate this.
        if (interfacename == "ompl_simplifier") {
            return boost::make_shared<or_ompl::OMPLSimplifier>(penv);
        }
        if (interfacename == "ompl_nothing") {
            return boost::make_shared<or_ompl::OMPLNothing>(penv);
        }

        // Check for automatically-wrapped optimizer (simplifier in the current terms).
        std::string const ompl_planner_name = interfacename.substr(5);
        for (std::string const &candidate_name : simplifier_names)
        {
            std::string candidate_name_lower = candidate_name;
            boost::algorithm::to_lower(candidate_name_lower);
            if (candidate_name_lower == ompl_planner_name) {
                or_ompl::PlannerFactory const factory = boost::bind(
                    &or_ompl::registry::createSimplifier, candidate_name, _1);
                return boost::make_shared<or_ompl::OMPLOptSimplifier>(penv, factory);
            }
        }

        // Check whether this is an automatically-wrapped planner.
        for (std::string const &candidate_name : planner_names) {
            std::string candidate_name_lower = candidate_name;
            boost::algorithm::to_lower(candidate_name_lower);

            if (candidate_name_lower == ompl_planner_name) {
                or_ompl::PlannerFactory const factory = boost::bind(
                    &or_ompl::registry::createPlanner, candidate_name, _1);
                return boost::make_shared<or_ompl::OMPLPlanner>(penv, factory);
            }
        }

    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO &info) {
    std::vector<std::string> const planner_names
        = or_ompl::registry::getPlannerNames();

    for (std::string const &planner_name : planner_names) {
        std::string const or_planner_name = "OMPL_" + planner_name;
        info.interfacenames[PT_Planner].push_back(or_planner_name);
    }
   
    std::vector<std::string> const simplifier_names = or_ompl::registry::getSimplifierNames();
    for (std::string const &sim_name : simplifier_names) {
        std::string const or_simplifier_name = "OMPL_" + sim_name;
        info.interfacenames[PT_Planner].push_back(or_simplifier_name);
    }

    info.interfacenames[PT_Planner].push_back("OMPL_Simplifier");
    info.interfacenames[PT_Planner].push_back("OMPL_Nothing");

    // Forward OMPL log messages to OpenRAVE.
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    ompl::msg::useOutputHandler(new or_ompl::OpenRAVEHandler);
}

RAVE_PLUGIN_API void DestroyPlugin() {
}
