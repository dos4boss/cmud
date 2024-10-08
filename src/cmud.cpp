#include <chrono>
#include <thread>
#include <string>
#include <fstream>

#include <cli/clilocalsession.h>
#include <cli/cli.h>
#include <cli/filehistorystorage.h>
#include <cli/clifilesession.h>
#include <gsl/gsl-lite.hpp>

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

  rootMenu->Insert("eep_read",
                   [](std::ostream &out, const std::string &board_name, const std::string &board_number,
                      const std::string &address, const std::string &length) {

                     std::uint_fast8_t board_number_;
                     try {
                       board_number_ = gsl::narrow<uint8_t>(std::stoul(board_number, 0, 0));
                     } catch (const std::exception &e) {
                       out << "board_number " << board_number<< " is invalid (" << e.what() << ")" << std::endl;
                       return;
                     }
                     std::uint_fast16_t address_;
                     try {
                       address_ = gsl::narrow<uint16_t>(std::stoul(address, 0, 0));
                     } catch (const std::exception &e) {
                       out << "address " << address << " is invalid (" << e.what() << ")" << std::endl;
                       return;
                     }

                     std::uint_fast16_t length_;
                     try {
                       length_ = gsl::narrow<uint16_t>(std::stoul(length, 0, 0));
                     } catch (const std::exception &e) {
                       out << "length " << length << " is invalid (" << e.what() << ")" << std::endl;
                       return;
                     }

                     const auto iter = std::find_if(board_interface::boards.begin(), board_interface::boards.end(),
                                                    [&](const std::reference_wrapper<const board_interface::Board> &board) {
                                                      return (board.get().name_ == board_name) && (board.get().number_ == board_number_);
                                                    });

                     if (iter == board_interface::boards.end()) {
                       out << "Board does not exist" << std::endl;
                       return;
                     }

                     const auto &result = iter->get().read_eeprom(address_, length_, out);
                     out << std::hex;
                     for(const auto &byte : result)
                       out << +byte << " ";
                     out << std::dec << std::endl;

                     std::ofstream file(board_name + "_" + board_number + "_" + address + "_" + length + ".bin", std::ofstream::binary);
                     file.write((const char*)result.data(), result.size());
                     file.close();
                   },
                   "read eeprom of board and board_idx at address for bytes");

  rootMenu->Insert("dig_init",
                   [](std::ostream &out, const std::string fpga_str, const std::string file_path) {
                     fpga_interface::load_fpga(fpga_str, file_path, out);
                   },
                   "Load bitstream to fpga");

  rootMenu->Insert("cor_gv",
                   [](std::ostream &out) {
                     board_interface::cor1_interface.get_version(out);
                   },
                   "Get correction processor firmware version");

  rootMenu->Insert("cor_gbi",
                   [](std::ostream &out) {
                     board_interface::cor1_interface.get_board_info(out);
                   },
                   "Get correction processor board info");

  rootMenu->Insert("cor_gs",
                   [](std::ostream &out, const std::string &rx_tx_str) {
                     mmio_interface::rx_tx rx_tx_;
                     if (rx_tx_str == "RX")
                       rx_tx_ = mmio_interface::rx_tx::RX;
                     else if (rx_tx_str == "TX")
                       rx_tx_ = mmio_interface::rx_tx::TX;
                     else {
                       out << "Invalid selection (" << rx_tx_str << "). It must be 'RX' or 'TX'.";
                       return;
                     }

                     board_interface::cor1_interface.get_status(rx_tx_, out);
                   },
                   "Get correction processor status");

  rootMenu->Insert("cor_gs2", [](std::ostream &out, const std::string &rx_tx_str) {
                                mmio_interface::rx_tx rx_tx_;
                                if (rx_tx_str == "RX")
                                  rx_tx_ = mmio_interface::rx_tx::RX;
                                else if (rx_tx_str == "TX")
                                  rx_tx_ = mmio_interface::rx_tx::TX;
                                else {
                                  out << "Invalid selection (" << rx_tx_str
                                      << "). It must be 'RX' or 'TX'.";
                                  return;
                                }
                                board_interface::cor1_interface.get_status_2(rx_tx_, out);
                              },
    "Get correction processor status 2");

  rootMenu->Insert("cor_sck", [](std::ostream &out) {
                                board_interface::cor1_interface.self_check(out);
                              },
    "Run correction processor self check");

  rootMenu->Insert("cor_wss",
                   [](std::ostream &out, const std::string &switch_id_str, const std::string &switch_state_str) {
                     board_interface::cor1_interface.write_switch_state(switch_id_str, switch_state_str, out);
                   },
                   "Write correction processor switch state");

  rootMenu->Insert("cor_rss",
                   [](std::ostream &out, const std::string &switch_id_str) {
                     board_interface::cor1_interface.read_switch_state(switch_id_str, out);
                   },
                   "Read correction processor switch state");

  rootMenu->Insert("cor_wsv",
                   [](std::ostream &out, const std::string &switch_id_str, const std::string &switch_state_str) {
                     board_interface::cor1_interface.write_switch_value(switch_id_str, switch_state_str, out);
                   },
                   "Write correction processor switch value");


  rootMenu->Insert("cor_rsv",
                   [](std::ostream &out, const std::string &switch_id_str) {
                     board_interface::cor1_interface.read_switch_value(switch_id_str, out);
                   },
                   "Read correction processor switch value");

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
