/*
 * cs.h
 *
 *  Created on: Nov 15, 2020
 *      Author: nathan
 */

#ifndef INC_CS_H_
#define INC_CS_H_


#define __HAL_ENTER_CRITICAL_SECTION()     \
   uint32_t PriMsk;                       \
   PriMsk = __get_PRIMASK();              \
     __set_PRIMASK(1);                    \


#define __HAL_EXIT_CRITICAL_SECTION()      \
   __set_PRIMASK(PriMsk);                 \


#endif /* INC_CS_H_ */
