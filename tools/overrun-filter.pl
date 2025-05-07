#!/usr/bin/perl
use strict;
use warnings;
@ARGV or @ARGV = "$ENV{HOME}/f25/some-overrun-times.txt";  # sorry, this file path is hardcoded for simon
while (<>) {
	if (/([a-zA-Z0-9.]+\(\)): ([0-9.]+)s$/) {
		my $fn = $1;
		my $time = $2;
		$time = int($time * 1000);
		if ($time > 10) {
			print "$time ms\t$fn\n";
		}
	}
}
