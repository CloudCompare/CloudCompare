// Example of a plugin action

#include "ccMainAppInterface.h"

namespace Example
{
	// This is an example of an action's method called when the corresponding action
	// is triggered (i.e. the corresponding icon or menu entry is clicked in CC's
	// main interface). You can access most of CC's components (database,
	// 3D views, console, etc.) via the 'appInterface' variable.
	void performActionA( ccMainAppInterface *appInterface )
	{
		if ( appInterface == nullptr )
		{
			// The application interface should have already been initialized when the plugin is loaded
			Q_ASSERT( false );
			
			return;
		}
		
		/*** HERE STARTS THE ACTION ***/
	
		// Put your code here
		// --> you may want to start by asking for parameters (with a custom dialog, etc.)
	
		// This is how you can output messages
		// Display a standard message in the console
		appInterface->dispToConsole( "[ExamplePlugin] Hello world!", ccMainAppInterface::STD_CONSOLE_MESSAGE );
		
		// Display a warning message in the console
		appInterface->dispToConsole( "[ExamplePlugin] Warning: example plugin shouldn't be used as is", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
		
		// Display an error message in the console AND pop-up an error box
		appInterface->dispToConsole( "Example plugin shouldn't be used - it doesn't do anything!", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
	
		/*** HERE ENDS THE ACTION ***/
	}
}
