apt install -y unzip ripgrep wget git
curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux64.tar.gz
rm -rf /opt/nvim
tar -C /opt -xzf nvim-linux64.tar.gz
echo 'export PATH="$PATH:/opt/nvim-linux64/bin"' >> ~/.bashrc
wget https://nodejs.org/dist/v20.17.0/node-v20.17.0-linux-x64.tar.xz
tar -C /opt -xvf node-v20.17.0-linux-x64.tar.xz 
echo 'export PATH="$PATH:/opt/node-v20.17.0-linux-x64/bin/"' >> ~/.bashrc
git clone https://github.com/quimortiz/kickstart.nvim.git "${XDG_CONFIG_HOME:-$HOME/.config}"/nvim


# Run this afterwards!
# souce ~/.bashrc 

