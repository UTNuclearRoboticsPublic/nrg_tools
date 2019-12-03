#pragma once

#include <sstream>

namespace nrg_tools{

    /**
     * Gets a string for printing a std::vector
     * @param input     The input vector
     * @return          A string of the vector, with brackets, commas, and proper spacing
     */
    template <class T> const std::string getStr(const std::vector<T> input)
    {
        std::stringstream stringstream;
        stringstream << "[";
        for(typename std::vector<T>::const_iterator it=input.begin(); it!=input.end(); ++it)
        {
            stringstream << *it << ", ";
        }

        stringstream.seekp(-2,stringstream.cur);
        stringstream << ']';

        std::string output = stringstream.str();
        output.erase(output.end()-1);
        return output;
    }

} //end nrg_tools namespace