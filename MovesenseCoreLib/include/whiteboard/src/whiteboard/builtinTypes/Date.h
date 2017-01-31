#pragma once
/******************************************************************************
Copyright (c) Suunto Oy 2015.
All rights reserved.
******************************************************************************/

#include "whiteboard/integration/port.h"
#include "whiteboard/builtinTypes/Structures.h"

namespace whiteboard
{

WB_STRUCT_BITFIELD_PACK_BEGIN()

/**
*	Date (DD.MM.YYYY)
*	Range: 1.1.1 - 31.12.4095
*	Precision: 1 day
*/
struct WB_STRUCT_BITFIELD_PACKED Date
{
    /** Default constructor */
    inline Date() : year(1), month(1), day(1){};

    /** Constructor that initializes all date fields.
    *
    * @param day	Day of month.
    * @param month	Month.
    * @param year	Year.
    */
    inline Date(const uint8 day_, const uint8 month_, const uint16 year_) : year(year_), month(month_), day(day_){};

    /** Checks if Date is valid.
    *
    * @return	True if Date is valid, otherwise false.
    */
    bool isValidDate() const
    {
        bool valid = true;
        if (year < 1 || year > 9999)
            valid = false;
        bool isLeapYear = (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
        switch (month)
        {
        // Months with 31 days.
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
            if (day < 1 || day > 31)
                valid = false;
            break;
        // February, either 28 or 29 days depending on leap year.
        case 2:
            if (isLeapYear && (day < 1 || day > 29))
                valid = false;
            else if (!isLeapYear && (day < 1 || day > 28))
                valid = false;
            break;
        // Months with 30 days.
        case 4:
        case 6:
        case 9:
        case 11:
            if (day < 1 || day > 30)
                valid = false;
            break;
        // Invalid month.
        default:
            valid = false;
            break;
        }
        return valid;
    };

#ifdef WB_STRUCTS_SUPPORT_BITFIELDS
    // Using bitfields to avoid alignment problems in target device.
    uint16 year : 12; // 0 - 4095
    uint16 month : 4;
    uint8 day : 5;
#else
    WB_ALIGNED(uint16) year;
    WB_ALIGNED(uint8) month;
    WB_ALIGNED(uint8) day;
#endif

    // Structure type identification and serialization
    typedef int Structure;
    static const LocalDataTypeId DATA_TYPE_ID = 24;
    static const StructureValueSerializer<Date> serializer;
    WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const StructureValueCleaner<Date> cleaner;)

    // Visitor pattern implementation
    inline void visit(IStructureVisitor& rVisitor) { rVisitor.visit(year).visit(month).visit(day); }
};

WB_STRUCT_BITFIELD_PACK_END()

} // namespace whiteboard
