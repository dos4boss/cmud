#include <chrono>
#include <thread>

#include <cli/clilocalsession.h>
#include <cli/cli.h>
#include <cli/filehistorystorage.h>
#include <cli/clifilesession.h>

#include "hw_interface_helpers.hpp"
#include "board_interface.hpp"
#include "fpga_interface.hpp"

int main(int argc, char *argv[]) {
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

  rootMenu->Insert("get_ch",
                   [](std::ostream &out) {
                     board_interface::check_for_present_boards(out);
                   },
                   "get ch");

  rootMenu->Insert("dig_init",
                   [](std::ostream &out, const std::string fpga_str, const std::string file_path) {
                     fpga_interface::load_fpga(fpga_str, file_path, out);
                   },
                   "Load bitstream to fpga");

  rootMenu->Insert("sleep", [](std::ostream &out, const unsigned seconds) {
    out << "Sleeping for " << seconds << " seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
  });

  //rootMenu->Insert("?", [&rootMenu](std::ostream& out) { rootMenu->Help(out); });


  // create a cli with the given root menu and a persistent storage
  // you must pass to FileHistoryStorage the path of the history file
  // if you don't pass the second argument, the cli will use a VolatileHistoryStorage object that keeps in memory
  // the history of all the sessions, until the cli is shut down.
  cli::Cli cli(std::move(rootMenu), std::make_unique<cli::FileHistoryStorage>(".cmud.history"));

  if(argc < 2) {
    cli::CliLocalTerminalSession localSession(cli, std::cout, 200);

    bool running = true;

    localSession.ExitAction([&running](auto& out)
                            {
                              out << "Thanks for using this cmud port. Exiting application now...\n";
                              running = false;
                            });

    while(running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
  }
  else {
    std::ifstream infile(argv[1]);
    if(!infile) {
      std::cout << "Could not open file '" << argv[1] << "'" << std::endl;
      return EXIT_FAILURE;
    }
    cli::CliFileSession fileSession(cli, infile, std::cout);
    fileSession.Start();
  }

  return EXIT_SUCCESS;
}
