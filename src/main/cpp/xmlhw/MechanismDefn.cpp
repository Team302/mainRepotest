
//====================================================================================================================================================
/// Copyright 2020 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//========================================================================================================
/// @class MechansimDefn
/// @brief Create a mechaism from an XML definition 
//========================================================================================================

// C++ Includes
#include <memory>
#include <string>
#include <utility>

// FRC includes
#include <frc/AnalogInput.h>
#include <frc/SmartDashboard/SmartDashboard.h>

// Team 302 includes
#include <subsys/MechanismFactory.h>
#include <subsys/interfaces/IMech.h>

#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/ServoMap.h>

#include <xmlhw/CanCoderDefn.h>
#include <xmlhw/DigitalInputDefn.h>
#include <xmlhw/MotorDefn.h>
#include <xmlhw/ServoDefn.h> 

#include <subsys/MechanismTypes.h>

#include <utils/Logger.h>

#include <xmlhw/MechanismDefn.h>
#include <hw/interfaces/IDragonMotorController.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse a Mechanism XML element and create an IMechanism from its definition.
/// @return IMech*   pointer to the mechanism
void MechanismDefn::ParseXML
(
    xml_node      mechanismNode
)
{

    // initialize attributes
    MechanismTypes::MECHANISM_TYPE type = MechanismTypes::UNKNOWN_MECHANISM;

    bool hasError       = false;

    // Parse/validate xml
    for (xml_attribute attr = mechanismNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        if ( strcmp( attr.name(), "type" ) == 0 )
        {
            string typeStr = attr.as_string();
            for_each( typeStr.begin(), typeStr.end(), [](char & c){c = ::toupper(c);});

            if ( typeStr.compare( "INTAKE") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::INTAKE;
            }
            else if ( typeStr.compare( "BALL_HOPPER") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::BALL_HOPPER;
            }
            else if ( typeStr.compare( "BALL_TRANSFER") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::BALL_TRANSFER;
            }
            else if ( typeStr.compare( "SHOOTER") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::SHOOTER;
            }
            else if ( typeStr.compare( "TURRET") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::TURRET;
            }
            else
            {
                    string msg = "unknown Mechanism type ";
                    msg += attr.value();
                    Logger::GetLogger()->LogError( "MechanismDefn::ParseXML", msg );
                    hasError = true;
            }
        }
        else
        {
            string msg = "invalid attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( "MechanismDefn::ParseXML", msg );
            hasError = true;
        }
    }

    // Parse/validate subobject xml
    unique_ptr<MotorDefn> motorXML = make_unique<MotorDefn>();
    unique_ptr<DigitalInputDefn> digitalXML = make_unique<DigitalInputDefn>();
    unique_ptr<ServoDefn> servoXML = make_unique<ServoDefn>();
    unique_ptr<CanCoderDefn> cancoderXML = make_unique<CanCoderDefn>();

    IDragonMotorControllerMap motors;
    ServoMap servos;
    DigitalInputMap digitalInputs;
    shared_ptr<ctre::phoenix::sensors::CANCoder> canCoder = nullptr;

    for (xml_node child = mechanismNode.first_child(); child  && !hasError; child = child.next_sibling())
    {
        if ( strcmp( child.name(), "motor") == 0 )
        {
            auto motor = motorXML.get()->ParseXML(child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
        }
        else if ( strcmp( child.name(), "digitalInput") == 0 )
        {
            auto digitalIn = digitalXML->ParseXML(child);
            if ( digitalIn.get() != nullptr )
            {
                digitalInputs[digitalIn.get()->GetType()] = digitalIn;
            }
        }
        else if ( strcmp( child.name(), "servo") == 0 )
        {
            auto servo = servoXML->ParseXML(child);
            if ( servo.get() != nullptr )
            {
                servos[servo.get()->GetUsage()] = servo;
            }
        }
        else if ( strcmp( child.name(), "cancoder" ) == 0)
        {
            canCoder = cancoderXML.get()->ParseXML(child);
        }
        else
        {
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogError( string("MechanismDefn"), msg );
        }
    }


    // create instance
    if ( !hasError )
    {
        MechanismFactory* factory =  MechanismFactory::GetMechanismFactory();
        factory->CreateIMechanism( type, motors, servos, digitalInputs, canCoder );
    }
}
