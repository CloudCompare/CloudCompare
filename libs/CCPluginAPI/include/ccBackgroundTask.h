#pragma once
// ##########################################################################
// #                                                                        #
// #                            CLOUDCOMPARE                                #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: CloudCompare project                      #
// #                                                                        #
// ##########################################################################

// Qt
#include <QEventLoop>
#include <QFutureWatcher>
#include <QtConcurrentRun>

// System
#include <type_traits>
#include <utility>

//! Helper to run a long computation without freezing the GUI
class ccBackgroundTask
{
  public:
	//! Runs a function in a worker thread while the GUI keeps refreshing
	/** Calling a long computation directly from the GUI thread blocks it. Progress
	    dialogs post their refresh to the GUI thread (see ccProgressDialog::update),
	    so while that thread is blocked nothing is repainted and the application
	    looks frozen, even though the progress value is being updated.

	    This runs 'function' in a worker thread and spins a local event loop until it
	    is done. Queued refreshes are then processed as they arrive, so the progress
	    dialog updates and the Cancel button stays usable. The call still only
	    returns once the function has finished, so callers don't need restructuring.

	    \warning the rest of the application also stays responsive while the function
	    runs, so the caller should make sure that starting another operation in the
	    meantime is either harmless or prevented.

	    \param function the function to run
	    \return whatever the function returns
	**/
	template <typename Func>
	static auto Run(Func&& function) -> decltype(function())
	{
		using ResultType = decltype(function());

		QFutureWatcher<ResultType> watcher;
		QEventLoop                 loop;

		// queued, so that the loop is still quit properly if the function happens to
		// finish before exec() has started
		QObject::connect(&watcher, &QFutureWatcherBase::finished, &loop, &QEventLoop::quit, Qt::QueuedConnection);

		watcher.setFuture(QtConcurrent::run(std::forward<Func>(function)));
		loop.exec();

		if constexpr (!std::is_void_v<ResultType>)
		{
			return watcher.result();
		}
	}
};
