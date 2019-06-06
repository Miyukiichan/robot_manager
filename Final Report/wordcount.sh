#!/bin/sh

pdftotext mmp-report.pdf -f $1 -l $2 - | sed "s/^[0-9]* of [0-9]*$//g" | wc -w
