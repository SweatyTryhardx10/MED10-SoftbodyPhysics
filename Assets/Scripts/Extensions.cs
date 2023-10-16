using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Extensions
{
    public static bool GetBit(this byte b, int bitNumber)
    {
        var bit = (b & (1 << bitNumber)) != 0;
        
        return bit;
    }
}
