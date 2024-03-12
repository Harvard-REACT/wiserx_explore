#include<explore/explore.h>

int main()
{
    std::string aoa_fn = "/home/react-ws-1/catkin_ws/src/wsr_exploration/data/aoa_val.csv";
    std::ifstream fin;
    time_t current_time_val, last_time;
    auto start_val = std::chrono::high_resolution_clock::now();
    auto stop_val = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    bool first_itr = true;

    while(true)
    {
        duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
        if(duration.count() > 3)
        {
            fin.open(aoa_fn);
            if(fin.is_open()) 
            {
                fin.seekg(-2,std::ios_base::end);                // go to one spot before the EOF

                bool keepLooping = true;
                while(keepLooping) {
                    char ch;
                    fin.get(ch);                            // Get current byte's data

                    if((int)fin.tellg() <= 1) {             // If the data was at or before the 0th byte
                        fin.seekg(0);                       // The first line is the last line
                        keepLooping = false;                // So stop there
                    }
                    else if(ch == '\n') {                   // If the data was a newline
                        keepLooping = false;                // Stop at the current position.
                    }
                    else {                                  // If the data was neither a newline nor at the 0 byte
                        fin.seekg(-2,std::ios_base::cur);        // Move to the front of that data, then to the front of the data before it
                    }
                }

                std::string lastLine; 
                std::vector <std::string> tokens;           
                getline(fin,lastLine);                      // Read the current line
                fin.close();
                
                // stringstream class check1
                std::stringstream check1(lastLine);
                std::string intermediate;
                
                // Tokenizing w.r.t. space ' '
                while(getline(check1, intermediate, ','))
                {
                    tokens.push_back(intermediate);
                }

                if(first_itr)
                {
                    last_time = strtoul( tokens[0].c_str(), NULL, 0 );
                    first_itr = false;
                }

                current_time_val = strtoul( tokens[0].c_str(), NULL, 0 );
                if( current_time_val > last_time )
                {
                    for(int i = 0; i < tokens.size(); i++)
                        std::cout << tokens[i] << '\n';
                    
                    last_time = current_time_val;
                }
                else
                {
                    std::cout << "Old data" << std::endl;
                }
            }
            start_val = std::chrono::high_resolution_clock::now();
        }
        stop_val = std::chrono::high_resolution_clock::now();
    }

    return 0;
}