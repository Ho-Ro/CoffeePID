#!/bin/sh

# go to www directory and zip all html files

cd data/www
gzip --force *.html
