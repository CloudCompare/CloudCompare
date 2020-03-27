// sip implementation of ScalarField

%ExportedHeaderCode
#include <ScalarField.h>
%End

namespace CCLib
{
    class ScalarField
    {
    %TypeHeaderCode
    #include <ScalarField.h>
    %End
    
    %Docstring
    A simple scalar field (to be associated to a point cloud)
    A monodimensional array of scalar values.
    Invalid values can be represented by NAN_VALUE.
    %End
    
    public:
        explicit ScalarField(const char* name = nullptr);
        
        const char* getName() const;
        
    protected:
        ~ScalarField();
    };
};
