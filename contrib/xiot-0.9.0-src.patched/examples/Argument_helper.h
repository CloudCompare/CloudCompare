/*
*
* Argument Helper
*
* Daniel Russel drussel@alumni.princeton.edu
* Stanford University
*
*
* This software is not subject to copyright protection and is in the
* public domain. Neither Stanford nor the author assume any
* responsibility whatsoever for its use by other parties, and makes no
* guarantees, expressed or implied, about its quality, reliability, or
* any other characteristic.
*
*/

#ifndef _DSR_ARGS_H_
#define _DSR_ARGS_H_
#include <vector>
#include <map>
#include <list>
#include <string>


 /*!
   
 \mainpage A Simple C++ Argument Parser

    This is a class to aid handling of command line arguments in a C++
    program. It follows (and enforces) the unsual conventions.

    New arguments are added by calling functions of the form of new_[optional_/named_]type.

    - type is the type of the value for the argument (int, double, string, vector of strings...)

    - optional means that the use doesn't have to supply it.

    - named means that it is identified by following an "-c" or
      "--long-name" identifier. All named arguments are optional.

    Unnamed arguments are expected to appear in order of addition on
    the command line. Named arguments can be passed in any order (and
    mixed with the unnamed arguments).

    The special argument "--" means that all remaining arguments are
    treated as unnamed (so you can pass file names that begin with -).

    When calling a new_foo function to create an argument the
    following can/must be passed in
    
    - For all arguments
     + The place to put the value.
     + A description of the value of the argument (i.e. it is a filename)
     + A description of the argument as a whole (i.e. it is the input file).

    - For named arguments
     + A character for the short name.
     + A string for the long name.

     When the program is called if "--help" is passed as an argument
     the useage information is printed and the program exits.

    There is always an implicit "-v" flag for verbose which sets the
    dsr::verbose variable and a "-V" which sets the VERBOSE variable.

    Any extra arguments or arguments with unexpected types are treated
    as errors and cause the program to abort. Extra arguments can be
    allowed for by adding a std::vector<std::string> to store them
    using the "set_string_vector" function. All extra (unnamed)
    arguments are placed there.

    This software is not subject to copyright protection and is in the
    public domain. Neither Stanford nor the author assume any
    responsibility whatsoever for its use by other parties, and makes no
    guarantees, expressed or implied, about its quality, reliability, or
    any other characteristic.

    An example using the class is:
    \include argument_helper_example.cc

  */

namespace dsr{
  extern bool verbose, VERBOSE;

  //! A helper class for parsing command line arguments.
  /*!
    This is the only class you need to look at in order to use it. 
  */
  class Argument_helper{
  private:
    class Argument_target;

 
    class FlagTarget;
    class DoubleTarget;
    class IntTarget;
    class UIntTarget;
    class StringTarget;
    class CharTarget;
    class StringVectorTarget;

  public:
    Argument_helper();
    //! Toggle a boolean
    void new_flag(char key, const char *long_name, const char *description,bool &dest);

    //! add a string argument
    void new_string( const char *arg_description, const char *description, std::string &dest);
    //! add a string which must have a key.
    void new_named_string(char key, const char *long_name,
			  const char *arg_description,
			  const char *description,  std::string &dest);
    //! Add an optional string-- any extra arguments are put in these.
    void new_optional_string( const char *arg_description, const char *description, std::string &dest);

    //! add an int
    void new_int( const char *arg_description, const char *description, int &dest);
    //! Add an int.
    void new_named_int(char key, const char *long_name,const char *value_name,
		       const char *description,
		       int &dest);
    //! Add an optional named int.
    void new_optional_int(const char *value_name,
			  const char *description,
			  int &dest);

    //! Add a named double.
    void new_double(const char *value_name,
		    const char *description,
		    double &dest);

    //! Add a named double.
    void new_named_double(char key, const char *long_name,const char *value_name,
			  const char *description,
			  double &dest);
    //! Add a named double.
    void new_optional_double(const char *value_name,
			     const char *description,
			     double &dest);

    //! Add an char.
    void new_char(const char *value_name,
				const char *description,
				char &dest);
    //! Add an optional char.
    void new_named_char(char key, const char *long_name,const char *value_name,
			   const char *description,
			   char &dest);
    //! Add an named char.
    void new_optional_char(const char *value_name,
			   const char *description,
			   char &dest);

    //! Add an unsigned int.
    void new_unsigned_int(const char *value_name, const char *description,
			  unsigned int &dest);
    //! Add an named unsigned int.
    void new_optional_unsigned_int(const char *value_name, const char *description,
				unsigned int &dest);
    //! Add an optional named unsigned int.
    void new_named_unsigned_int(char key, const char *long_name,
				   const char *value_name, const char *description,
				   unsigned int &dest);


    //! add a target which takes a list of strings
    /*!
      Only named makes sense as the string vector default handles unnamed and optional.
    */
    void new_named_string_vector(char key, const char *long_name,
				 const char *value_name, const char *description,
				 std::vector<std::string> &dest);

    //! add a vector of strings.
    /*!  Any arguments which are not claimed by earlier unnamed
      arguments or which are named are put here. This means you cannot
      have a string vector followed by a string.
    */
    void set_string_vector(const char *arg_description, const char *description, std::vector<std::string> &dest);

    //! Set who wrote the program.
    void set_author(const char *author);

    //! Set what the program does.
    void set_description(const char *descr);

    //! Set what the version is.
    void set_version(float v);
    void set_version(const char *str);

    //! Set the name of the program.
    void set_name(const char *name);

    //! Set when the program was built.
    void set_build_date(const char *date);

    //! Process the list of arguments and parse them.
    /*!
      This returns true if all the required arguments are there. 
    */
    void process(int argc, const char **argv);
    void process(int argc, char **argv){
      process(argc, const_cast<const char **>(argv));
    }
    //! Write how to call the program.
    void write_usage(std::ostream &out) const;
    //! Write the values of all the possible arguments.
    void write_values(std::ostream &out) const;
    
    ~Argument_helper();
  protected:
    typedef std::map<char, Argument_target*> SMap;
    typedef std::map<std::string, Argument_target*> LMap;
    typedef std::vector<Argument_target*> UVect;
    // A map from short names to arguments.
    SMap short_names_;
    // A map from long names to arguments.
    LMap long_names_;
    std::string author_;
    std::string name_;
    std::string description_;
    std::string date_;
    float version_;
    bool seen_end_named_;
    // List of unnamed arguments
    std::vector<Argument_target*> unnamed_arguments_;
    std::vector<Argument_target*> optional_unnamed_arguments_;
    std::vector<Argument_target*> all_arguments_;
    std::string extra_arguments_descr_;
    std::string extra_arguments_arg_descr_;
    std::vector<std::string> *extra_arguments_;
    std::vector<Argument_target*>::iterator current_unnamed_;
    std::vector<Argument_target*>::iterator current_optional_unnamed_;
    void new_argument_target(Argument_target*);
    void handle_error() const;
  private:
    Argument_helper(const Argument_helper &){};
    const Argument_helper& operator=(const Argument_helper &){return *this;}
  };
  
}

#endif
