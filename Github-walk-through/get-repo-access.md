# Get Repository Access

## Basic Getting Access

* [follow github checking for keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)
* [follow github generating new ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
* [follow github add ssh key to account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
* [test your ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/testing-your-ssh-connection)

## Advanced extras

### Add key to config

#### Find SSH Directory

Windows : $HOME\.ssh (powershell)

OSX : ~/.ssh (may need to create)

Linux : Try ~/.ssh

#### Create or Modify Config File

you are looking to modify a file named *config* in your ssh folder if it does not exist then create it.

#### Add a Host for Github servers

The file should have the host formated as follows replacing 'value' with desired value.
ps. keep an eye out for trailing spaces

```config
Host 'name of host'
    HostName '000.111.22.44'
    User 'UserName'
    IdentityFile '~/.ssh/generated_key'
    PubkeyAuthentication yes
```

##### Example of created Host

```config
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/generated_key
    PubkeyAuthentication yes
```

##### Details

* Host will be what you call ex. (`ssh jelly`)
* HostName is how your computer finds the host usualy ip
* User is the username you use to sign in
* IdentityFile is the path to the key you are using
* PubkeyAuthentication is letting ssh know to use that provided key

## Trouble Shooting

If a git command is failing try running with [GIT_TRACE](https://git-scm.com/book/en/v2/Git-Internals-Environment-Variables)
> GIT_TRACE=(1 or 2) 'git command'
