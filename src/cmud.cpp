#include <chrono>
#include <thread>

#include <cli/clilocalsession.h>
#include <cli/cli.h>
#include <cli/filehistorystorage.h>

#include "hw_interface_helpers.hpp"

int main()
{
  hw_interface::init();

  cli::SetColor();

  auto rootMenu = std::make_unique<cli::Menu>("cmud");

  rootMenu->Insert("hsd_rd",
                   [](std::ostream &out, const std::string switch_name) {
                     hw_interface::read_switch(switch_name, out);
                   },
                   "Read switch");

  rootMenu->Insert("hsd_wr",
                   [](std::ostream &out, const std::string switch_name, const std::string switch_state) {
                     hw_interface::write_switch(switch_name, switch_state, out);
                   },
                   "Write switch");

  //rootMenu->Insert("?", [&rootMenu](std::ostream& out) { rootMenu->Help(out); });


  // create a cli with the given root menu and a persistent storage
  // you must pass to FileHistoryStorage the path of the history file
  // if you don't pass the second argument, the cli will use a VolatileHistoryStorage object that keeps in memory
  // the history of all the sessions, until the cli is shut down.
  cli::Cli cli(std::move(rootMenu), std::make_unique<cli::FileHistoryStorage>(".cmud.history"));

  cli::CliLocalTerminalSession localSession(cli, std::cout, 200);

  bool running = true;

  localSession.ExitAction([&running](auto& out)
                          {
                            out << "Thanks for using this cmud port. Exiting application now...\n";
                            running = false;
                          });

  while(running)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
