mod frontend;
mod chip8;

fn main() {
  let args :Vec<String> = std::env::args().collect();
  assert!(args.len() > 1, "Please provide a valid file path");
  let game = std::path::Path::new(&args[1]);
  assert!(game.is_file(), "Please provide a valid file path");
  let mut frontend = frontend::Frontend::new();
  frontend.load_game(game);
}
