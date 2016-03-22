/**
 * Overtaker - Sample application for calculating steering and acceleration commands.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef OVERTAKER_H_
#define OVERTAKER_H_

#include "core/base/ConferenceClientModule.h"

namespace msv {

    using namespace std;

    /**
     * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
     */
    class Overtaker : public core::base::ConferenceClientModule {
        private:
            /**
             * "Forbidden" copy constructor. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the copy constructor.
             *
             * @param obj Reference to an object of this class.
             */
            Overtaker(const Overtaker &/*obj*/);

            /**
             * "Forbidden" assignment operator. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the assignment operator.
             *
             * @param obj Reference to an object of this class.
             * @return Reference to this instance.
             */
            Overtaker& operator=(const Overtaker &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            Overtaker(const int32_t &argc, char **argv);

            virtual ~Overtaker();

            core::base::ModuleState::MODULE_EXITCODE body();
            double angleDifference(double initialHeading, double heading);

        private:
            virtual void setUp();

            virtual void tearDown();
            
            static const int m_followLane = 1;
            static const int m_turnOut = 2;
            static const int m_straighten = 3;
            static const int m_turnBack = 4;
            static const int m_returnNormal = 5;

            int m_counter;
            double m_steering;
            int m_sensor;
            double m_carLength;
            double m_speed;

    };

} // msv

#endif /*Overtaker_H_*/
