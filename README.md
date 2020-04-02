# MTBA6MicroinfPinguins
Project for the course Microinformatique with the robot ePuck2.

# Instructions for using this repo
The lib.zip needs to be extracted at the root, but it will not be watched (excluded in gitignore) by git as it has lots of files that need never change, so it might cause havoc if there was an issue with it.

# Conventions for code

## Functions and variables names
Without _ but with capitals between words.
For functions in each file a prefix for the file ex for file prefix tst and function name "do this now" : tstDoThisNow


# Conventions for code comments

## Separators in files between topic grouped code segments

/*===========================================================================*/
/* SERIAL_USB driver related setting.                                        */
/*===========================================================================*/

## Description of a function, only use the @ necessary (maybe just the @brief)
/**
 * @brief   Brief description.
 * @details More details if needed
 * @note    There can be multiple notes if needed
 * @warning If there should be attention given to something
 * 
 * @parameter [in] name_of_variable1 If there is an input param, brief description.
 * @parameter [in] name_of_variable2 If there is a second input param, brief description. Etc
 * @parameter [out] variable_name If there are params given but for the value after return (so has to be a pointer).
 * 
 * @return Description of what is returned (if anything).
*/
type functionName(type name_of_variable1, type name_of_variable2, type out_variable_name);

## Source file beginning
/*
 * fileName.extension
 *
 *  Created on: Month numDay, numYear
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Description of the file if necessary
 * Functions prefix for this file: prefixExample
 */

## Header file structure
/*
 * fileName.extension
 *
 *  Created on: Apr 2, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Description of the file if necessary
 */

#ifndef FILENAME_H_
#define FILENAME_H_


#endif /* FILENAME_H_ */



# Tips to remember

## Mac show/hide hidden files

On Mac, in order to show hidden files (files that start with a . are hidden by default on mac, so .gitignore for example) :
Open up your folder
Press Command+Shift+Dot
Your hidden files will become visible. Repeat step 2 to hide them again!

