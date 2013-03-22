#!/bin/sh

file=$1

cp "$file" "$file".bak
cat ../templates/header.tpl "$file".bak > "$file"
