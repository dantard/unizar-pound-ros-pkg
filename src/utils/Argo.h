#ifndef ARGO
#define ARGO
#include <iomanip>
#include <iostream>
#include <cxxabi.h>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

class Argo{

    std::vector<option> ops;
    std::string opts;

    class Param{
    public:
        enum types_t {INT, FLOAT, STRING, BOOL};
    protected:
        types_t type;
        bool has_parameter, mandatory;
        std::string description, long_name;
        std::string hr_type;
        char name;
    public:
        void init(char name, const char * long_name, bool has_parameter, bool mandatory, const char * description){
            this->name = name;
            if (description!=0){
                this->description = std::string(description);
            }
            this->has_parameter = has_parameter;
            this->mandatory = mandatory;
            if (long_name!=0){
                this->long_name = std::string(long_name);
            }
        }
        char getName(){
            return name;
        }
        std::string getLongName(){
            return long_name;
        }
        std::string setLongName(std::string long_name){
            this->long_name = long_name;
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
        virtual void processYAML(YAML::Node & config, bool override){}
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
        GenParam(char name, const char* long_name, T & thevalue, T dflt, bool has_parameter, bool mandatory, const char * description): value(thevalue){
            this->dflt = dflt;
            this->value = dflt;
            init(name, long_name, has_parameter, mandatory, description);
            setHRType();
        }
        virtual void setValue(std::string s){
            std::istringstream iss(s);
            iss >> value;
        }

        virtual std::string getHRValue(){
            std::ostringstream oss;
            oss << name << " = " << std::left<< std::setw(20) << value;
            if (long_name.compare("")!=0) {
                oss << " (" << long_name << ")";
            }
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
            // const char * p= typeid(T).name();
            // return demangle(p);
            return hr_type;
        }

        void processYAML(YAML::Node & config, bool override){
            if (YAML::Node parameter = config[long_name]){
                if (value == dflt || override){
                    value = parameter.as<T>();
                }
            }
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
            if (p->isMandatory()){
                std::cerr << "* ";
            }else{
                std::cerr << "  ";
            }
            std::cerr << "-" <<  p->getName();
            if (p->getLongName()!=""){
                std::cerr << ", --" << std::left<< std::setw(maxlen)  <<p->getLongName();
            }else{
                std::cerr << "    " << std::left<< std::setw(maxlen)  << "";
            }
            if (p->hasParameter()){
                std::cerr << " " << p->getHRType() << " : ";
            }else{
                std::cerr << "         : ";
            }

            std::cerr << p->getDescription() <<std::endl;
        }
        if (example.compare("")!=0){
            std::cerr << "Example: " << example << std::endl;
        }
    }

    template <class T> int add(char name, const char * long_name,T & value, T dflt, bool has_parameter, bool mandatory, const char * description){
        GenParam<T> *p = new GenParam<T>(name, long_name, value, dflt, has_parameter, mandatory, description);
        if (long_name != 0){
            struct option op;
            maxlen = strlen(long_name) > maxlen ? strlen(long_name) : maxlen;
            op.has_arg = has_parameter ? required_argument : no_argument;
            op.name  = long_name;
            op.val = 0; op.flag = 0;
            ops.push_back(op);
        }
        vec.push_back(p);
        opts += name;
        if(has_parameter){
            opts+=":";
        }
        return vec.size() - 1;
    }
    void addSwitch(char name, const char * long_name, bool & value, const char * description){
        add<bool>(name, long_name, value, false, false, false, description);
    }
    void addSwitchInt(char name, const char * long_name, int & value, const char * description){
        add<int>(name, long_name, value, 0, false, false, description);
    }
    void addSwitch(char name, bool & value, const char * description){
        add<bool>(name, 0, value, false, false, false, description);
    }
    void addInt(char name, const char * long_name, int & value, int dflt, const char * description){
        add<int>(name, long_name, value, dflt, true, false, description);
    }
    void addIntMandatory(char name, const char * long_name, int & value, int dflt, const char * description){
        add<int>(name, long_name, value, dflt, true, true, description);
    }
    void addDouble(char name, const char * long_name, double & value, int dflt, const char * description){
        add<double>(name, long_name, value, dflt, true, false, description);
    }
    void addString(char name, const char * long_name, std::string & value, std::string dflt, const char * description){
        add<std::string>(name, long_name, value, dflt, true, false, description);
    }

    void process(int argc, char * argv[]){

        int verbose = 0;

        std::vector<int> switches;

        while (1) {
            int option_index = 0;
            int curind = optind;
            int c = getopt_long(argc, argv, opts.c_str(),
                                &ops[0], &option_index);
            if (c == -1){
                break;
            }else if(c == 'v'){
                verbose = 1;
                continue;
            }else if(c == 'h'){
                showList();
                exit(0);
            }else if(c == '?'){
                std::cerr << "*** Unrecognized option '"<< argv[curind] << "'"<<std::endl;
                showList();
                exit(0);
            }else if(c == ':'){
                std::cerr << "*** Missing parameter for '" << argv[curind] << "'" <<std::endl;
                showList();
                exit(0);
            }

            for (int i = 0; i< vec.size(); i++){
                Param * p = vec[i];
                if(p->getName() == c || (c==0 && p->getLongName().compare(ops[option_index].name) == 0)){
                    if (p->hasParameter()){
                        p->setValue(std::string(optarg));
                    }else{
                        p->setValue("1");
                    }
                    switches.push_back(i);
                    break;
                }
            }
        }

        /* check dependencies */
        for (int i = 0; i<switches.size(); i++){
            std::map<int, std::vector<int> >::iterator mit = deps.find(switches[i]);
            if (mit != deps.end()){
                for (int j = 0; j< mit->second.size(); j++){

                    std::vector<int>::iterator vit = std::find(switches.begin(), switches.end(), abs(mit->second[j]));
                    if (mit->second[j] >= 0){
                        if (vit == switches.end()){
                            std::cerr << "*** Option '" << vec[mit->first]->getName() << "' must be used in conjunction with '" << vec[mit->second[j]]->getName() << "'" << std::endl;
                            showList();
                            exit(0);
                        }
                    }else{
                        if (vit != switches.end()){
                            std::cerr << "*** Option '" << vec[mit->first]->getName() << "' is incompatible with '" << vec[abs(mit->second[j])]->getName() << "'" << std::endl;
                            showList();
                            exit(0);
                        }
                    }
                }
            }
        }

        if (verbose){
            for (int j = 2; j< vec.size(); j++){
                std::cerr << vec[j]->getHRValue() << std::endl;
            }
        }
    }
    YAML::Node config;
    YAML::Node & processYAML(std::string & config_file, bool override = false){
        config = YAML::LoadFile(config_file);
        for (int j = 0; j< vec.size(); j++){
            vec[j]->processYAML(config, override);
        }
        return config;
    }

    void showValues(){
        for (int j = 2; j< vec.size(); j++){
            std::cerr << vec[j]->getHRValue() << std::endl;
        }

    }

    void addExample(std::string example){
        this->example = example;
    }

    std::map<int, std::vector<int> > deps;
    void addDependency(char c1, char c2, bool dual){
        int dependent = -1;
        int master = -1;
        for (int i = 0; i<vec.size(); i++){
            if (vec[i]->getName() == c1){
                dependent= i;
            }
            if (vec[i]->getName() == c2){
                master = i;
            }
        }
        assert(dependent!=-1 && master != -1);
        deps[dependent].push_back(master);
        if (dual){
            deps[master].push_back(dependent);
        }
    }

    void addIncompatibility(char c1, char c2){
        int dependent = -1;
        int master = -1;
        for (int i = 0; i<vec.size(); i++){
            if (vec[i]->getName() == c1){
                master = i;
            }
            if (vec[i]->getName() == c2){
                dependent = i;
            }
        }
        assert(dependent!=-1 && master != -1);
        deps[dependent].push_back(-master);
        deps[master].push_back(- dependent);
    }

    Argo(){
        maxlen = 0;
        opts = ":";
        add<int>('h', "help", helper,0, false, false, "Show this screen");
        add<int>('v', "verbose",helper,0, false, false, "Show values assigned to parameters");
    }
};

#define def_add_int(argo,param,param_long,var,dflt,desc) int var; argo.addInt(param, param_long, var, dflt, desc);
#define def_add_string(argo,param,param_long,var,dflt,desc) std::string var; argo.addString(param, param_long, var, dflt, desc);
#define def_add_switch(argo,param,param_long,var,desc) bool var; argo.addSwitch(param, param_long, var, desc);
#endif
