#ifndef ARGO
#define ARGO
#include <iomanip>
#include <iostream>
#include <cxxabi.h>

class Argo{

    class Param{
    public:
        enum types_t {INT, FLOAT, STRING, BOOL};
    protected:
        types_t type;
        bool has_parameter, mandatory;
        std::string name, description;
        std::string hr_type;
    public:
        void init(std::string name, std::string description, bool has_parameter, bool mandatory){
            this->name = name;
            this->description = description;
            this->has_parameter = has_parameter;
            this->mandatory = mandatory;
        }
        std::string getName(){
            return name;
        }
        std::string getDescription(){
            return description;
        }
        bool isMandatory(){
            return mandatory;
        }
        bool hasParameter(){
            return has_parameter;
        }
        virtual std::string getHRValue(){
        }
        virtual std::string getHRType(){
        }
        types_t getType(){
            return type;
        }
        virtual void setValue(std::string ){}
    };

    template <class T> class GenParam :public Param{
        T & value;
        T dflt;
        void setHRType(){
            const std::type_info & t = typeid(T);
            if (t == typeid(int)
                    || t == typeid(unsigned int)
                    || t == typeid(char)
                    || t == typeid(unsigned char)
                    || t == typeid(short)
                    || t == typeid(unsigned short)
                    || t == typeid(long int)
                    || t == typeid(unsigned long long int)
                    || t == typeid(long long int)
                    || t == typeid(unsigned long int)){
                hr_type = "integer";
                type = INT;
            }else if (t == typeid(double) || t == typeid(float)){
                hr_type = "float  ";
                type = FLOAT;
            }else if (t == typeid(bool)){
                hr_type = "boolean";
                type = BOOL;
            }else if (t == typeid(std::string)){
                hr_type = "string ";
                type = STRING;
            }
        }
    public:
        GenParam(T & thevalue, T dflt, std::string name, std::string description, bool has_parameter, bool mandatory): value(thevalue){
            this->dflt = dflt;
            this->value = dflt;
            init(name, description, has_parameter, mandatory);
            setHRType();
        }
        virtual void setValue(std::string s){
            std::istringstream iss(s);
            iss >> value;
        }
        virtual std::string getHRValue(){
            std::ostringstream oss;
            oss << name <<" = " << value ;
            return oss.str();
        }
        std::string demangle(const char* mangled)
        {
              int status;
              std::unique_ptr<char[], void (*)(void*)> result(
              abi::__cxa_demangle(mangled, 0, 0, &status), std::free);
              return result.get() ? std::string(result.get()) : "error occurred";
        }
        std::string getHRType(){
//            const char * p= typeid(T).name();
//            return demangle(p);
            return hr_type;
        }
    };
    std::vector<Param * > vec;
    int helper, maxlen;
    std::string example;
public:

    void showList(){
        std::cerr << "List of all options (* are mandatory):" << std::endl;
        for (int j = 0; j< vec.size(); j++){
            Param * p = vec[j];
            std::cerr << " " << std::left<< std::setw(maxlen+1) << p->getName();
            if (p->hasParameter()){
                std::cerr << " " << p->getHRType();
            }else{
                std::cerr << "        ";
            }
            if (p->isMandatory()){
                std::cerr << " *: ";
            }else{
                std::cerr << "  : ";
            }
            std::cerr << p->getDescription() <<std::endl;
        }
        if (example.compare("")!=0){
            std::cerr << "Example: " << example << std::endl;
        }
    }


    template <class T> void add(T & value, T dflt, std::string name, std::string description, bool has_parameter, bool mandatory){
        GenParam<T> *p = new GenParam<T>(value, dflt, "-" + name, description, has_parameter, mandatory);
        maxlen = name.size()>maxlen?name.size():maxlen;
        vec.push_back(p);
    }
    void addSwitch(bool & value, bool dflt, std::string name, std::string description){
        add<bool>(value, dflt,name, description, false, false);
    }
    void addStringMandatory(std::string & value, std::string name, std::string description){
        add<std::string>(value, "", name, description, true, true);
    }
    void addIntMandatory(int & value, std::string name, std::string description){
        add<int>(value, 0, name, description, true, true);
    }
    void addInt(int & value, int dflt, std::string name, std::string description){
        add<int>(value, dflt, name, description, true, false);
    }
    void addDouble(double & value, double dflt, std::string name, std::string description){
        add<double>(value, dflt, name, description, true, false);
    }
    void addString(std::string & value,std::string  dflt, std::string name, std::string description){
        add<std::string>(value, dflt, name, description, true, false);
    }
    void process(int argc, char * argv[]){

        for (int i = 1; i< argc; i++){
            if(strcmp(argv[i],"-h") == 0){
                showList();
                exit(0);
            }
        }

        for (int j = 1; j< argc; j++){
            if (argv[j][0]=='-'){
                bool found = 0;
                for (int i = 0; i< vec.size(); i++){
                    Param * p = vec[i];
                    if(p->getName().compare(argv[j]) == 0){
                        found = 1;
                    }
                }
                if (! found){
                    std::cerr << "*** Non existent switch " << argv[j] << std::endl;
                    if (atoi(argv[j])!=0){
                        std::cerr << "*** To insert negative numbers use '+' prefix, e.g. +" << argv[j] << std::endl;
                    }
                    showList();
                    exit(0);
                }
            }else if(argv[j][0]=='+'){
                argv[j][0] = ' ';
            }
        }

        for (int i = 0; i< vec.size(); i++){
            bool found = 0;
            Param * p = vec[i];
            for (int j = 1; j< argc; j++){
                if(p->getName().compare(argv[j]) == 0){
                    found = 1;
                    if (p->hasParameter()){
                        if (j+1 >= argc){
                            std::cerr << "*** It seems you forgot the parameter for " << p->getName() << std::endl;
                            showList();
                            exit(0);
                        }else{
                            for (int k = 1; k< vec.size(); k++){
                                Param * q = vec[k];
                                if(q->getName().compare(argv[j+1]) == 0){
                                    std::cerr << "*** It seems you forgot the parameter for " << p->getName() << std::endl;
                                    showList();
                                    exit(0);
                                }
                            }
                        }
                        p->setValue(std::string(argv[j+1]));
                    }else{
                        p->setValue("1");
                    }
                }
            }
            if (p->isMandatory() && found == 0){
                std::cerr << "*** Parameter for " << p->getName() << " is mandatory" << std::endl;
                showList();
                exit(0);
            }
        }
        for (int i = 1; i< argc; i++){
            if(strcmp(argv[i],"-v") == 0){
                for (int j = 2; j< vec.size(); j++){
                    std::cerr << vec[j]->getHRValue() << std::endl;
                }
            }
        }
    }

    void addExample(std::string example){
        this->example = example;
    }
    Argo(){
        maxlen = 0;
        add<int>(helper,0,"h", "Show this screen", false, false);
        add<int>(helper,0,"v", "Show values assigned to parameters", false, false);
    }
};
#endif
