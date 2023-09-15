#include "CSVReader.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
CSVReader::CSVReader(std::string filename)
{
    //Load CSV Contents into data vector, escaping quotes if necessary

    //Open file
    std::ifstream file(filename);
    //Check if file is open
    if(!file.is_open()) throw std::runtime_error("Could not open file");
    //Create string for line
    std::string line;
    //Loop through lines
    while(std::getline(file, line))
    {
        //Create vector for data
        std::vector<std::string> row;
        //Create string for field
        std::string field;
        //Create boolean for if we are inside a quoted field
        bool in_quotes = false;
        //Loop through characters in line
        for(char c : line)
        {
            //Check if we are in a quoted field
            if(in_quotes)
            {
                //Check if this is a quote
                if(c == '"')
                {
                    //Check if next character is also a quote
                    if(*(line.begin() + (line.find(c) + 1)) == '"')
                    {
                        //Add quote to field
                        field += '"';
                        //Increment iterator
                        line.begin()++;
                    }
                    else
                    {
                        //We are no longer in a quoted field
                        in_quotes = false;
                    }
                }
                else
                {
                    //Add character to field
                    field += c;
                }
            }
            else
            {
                //Check if this is a quote
                if(c == '"')
                {
                    //We are now in a quoted field
                    in_quotes = true;
                }
                else if(c == ',')
                {
                    //Add field to row
                    row.push_back(field);
                    //Clear field
                    field.clear();
                }
                else
                {
                    //Add character to field
                    field += c;
                }
            }
        }
        //Add field to row
        row.push_back(field);
        //Add row to data
        data.push_back(row);
    }
    //Close file
    file.close();
}
const vector<vector<string>>& CSVReader::getData()
{
    return data;
}
