#!/bin/zsh

perl -0777 -pi.bak -e '$_ =~ s{^/\*.*?\*/.*?#ifndef}{#ifndef}s' *.h || echo "No h"
perl -0777 -pi.bak -e '$_ =~ s{^/\*.*?\*/.}{}s' *.cpp || echo "No cpp"
perl -0777 -pi.bak -e '$_ =~ s{^/\*.*?\*/.}{}s' *.hpp || echo "No hpp."
