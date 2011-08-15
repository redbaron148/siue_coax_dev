#include <iostream>
#include <string>

#define RED     2
#define GREEN   01
#define BLUE    00
#define YELLOW  3

using namespace std;

bool is_valid_input(string input);
bool is_valid_char(char input);
int sequence_id_from_string(string input);

int main()
{
    string input = "";
    char bob = 32;
    bool valid = false;
    
    do
    {
        do
        {   
            cout << "input color sequence: ";
            cin >> input;
            if(input == "q") return 0;
        }while(!is_valid_input(input));
        
        cout << "id: " << sequence_id_from_string(input) << endl;
    }while(1);
}

bool is_valid_input(string input)
{
    if(!(input.length() == 1 || input.length() == 4))
        return false;
    for(int i = 0;i<input.length();i++)
        if(!is_valid_char(input[i])) return false;
    return true;
}

bool is_valid_char(char input)
{
    if(input == 'r' || input == 'g' || input=='b' || input=='y')
        return true;
    return false;
}

int sequence_id_from_string(string input)
{
    int color = 0;
    for(int i = 0;i<4;i++)
    {
        if(input[i] == 'r') color |= RED << (3-i)*2;
        else if(input[i] == 'g') color |= GREEN << (3-i)*2;
        else if(input[i] == 'b') color |= BLUE << (3-i)*2;
        else color |= YELLOW << (3-i)*2;
    }
    return color;
}
