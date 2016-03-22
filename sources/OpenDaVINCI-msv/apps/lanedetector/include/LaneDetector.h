/**
 * lanedetector - Sample application for detecting lane markings.
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

#ifndef LANEDETECTOR_H_
#define LANEDETECTOR_H_

#include <opencv/cv.h>
#include "core/SharedPointer.h"
#include "core/base/ConferenceClientModule.h"
#include "core/wrapper/SharedMemory.h"

#include "Lines.h"

namespace msv {

    using namespace std;

    /**
     * This class is an exemplary skeleton for processing video data.
     */
    class LaneDetector: public core::base::ConferenceClientModule {
        private:
	        /**
	         * "Forbidden" copy constructor. Goal: The compiler should warn
	         * already at compile time for unwanted bugs caused by any misuse
	         * of the copy constructor.
	         *
	         * @param obj Reference to an object of this class.
	         */
	        LaneDetector(const LaneDetector &/*obj*/);

	        /**
	         * "Forbidden" a ssignment operator. Goal: The compiler should warn
	         * already at compile time for unwanted bugs caused by any misuse
	         * of the assignment operator.
	         *
	         * @param obj Reference to an object of this class.
	         * @return Reference to this instance.
	         */
	        LaneDetector& operator=(const LaneDetector &/*obj*/);

        public:
	        /**
	         * Constructor.
	         *
	         * @param argc Number of command line arguments.
	         * @param argv Command line arguments.
	         */
	        LaneDetector(const int32_t &argc, char **argv);

	        virtual ~LaneDetector();

	        core::base::ModuleState::MODULE_EXITCODE body();

        protected:
	        /**
	         * This method is called to process an incoming container.
	         *
	         * @param c Container to process.
	         * @return true if c was successfully processed.
	         */
	        bool readSharedImage(core::data::Container &c);

        private:
	        bool m_hasAttachedToSharedImageMemory;
	        core::SharedPointer<core::wrapper::SharedMemory> m_sharedImageMemory;
	        IplImage *m_image;
	        IplImage *merge_image;
            bool m_debug;

			/* Scans for two valid lines in a vector of lines */
			void validLines(std::vector<Lines>& lines, int LorR);
            
            /* Measures the distance to full-white lines */
			double measureDistance(int yPos, int dir, IplImage* image);

			/* Measures the angle between delta X and delta Y */
			double measureAngle(IplImage *image);

			void setLines(IplImage* image);

			 /*---Translate a given point around (nearly) the middle of the image using a slightly offset 
    		vanishing point---*/
			int translatePoint(int yPos, int xPos);

	        virtual void setUp();

	        virtual void tearDown();

            void processImage();
            bool isEmpty(std::vector<Lines>& lines);

        	Lines upline1;
        	Lines upline2;

        	int state;
        	int counter;
        	int critAngleCounter;
        	int yCount;

        	int imgWidth;
        	int imgHeight;

        	const double SPEED;
        	const int SIZE;
        	
      		double tempAngle;

        	double critAngleRight;
        	double critAngleLeft;
        	int vanY;

        	double rightError;
        	double leftError;
        	int leftLength;
        	int rightLength;

			vector<Lines> rightList;
      		vector<Lines> leftList;
      		vector<Lines> validLeft;
      		vector<Lines> validRight;



	};

} // msv

#endif /*LANEDETECTOR_H_*/
