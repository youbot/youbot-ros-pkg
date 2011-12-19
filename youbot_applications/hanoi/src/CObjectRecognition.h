/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Steffen Waeldele
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef COBJECTRECOGNITION_H_
#define COBJECTRECOGNITION_H_

class CObjectRecognition {
public:
	CObjectRecognition();
	virtual ~CObjectRecognition();

	void getObjectList();
	void getObjectPos(int objectID);
};

#endif /* COBJECTRECOGNITION_H_ */
