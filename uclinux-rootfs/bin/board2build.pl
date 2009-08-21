#!/usr/bin/perl -w

my $board = $ARGV[0];

if(defined($ARGV[1])) {
	# grudgingly accept: board2build.pl BOARD REV
	$board .= $ARGV[1];
}

if(! defined($board)) {
	print "usage: board2build.pl <boardname>\n";
	print "\n";
	print "examples:\n";
	print "  board2build.pl 97456d0\n";
	print "  board2build.pl 97466b0\n";
	print "  board2build.pl 97420a0\n";
	exit 0;
}

$board =~ tr/A-Z/a-z/;
$board =~ s/7420dvr2/7420/;

if($board =~ m/^9?([0-9]{4,5})([a-z])([0-9])$/) {
	$chip = $1;
	$rev = $2."0";
} elsif($board =~ m/^9?([0-9]{4,5})$/) {
	$chip = $1;
	$rev = "a0";
} else {
	print "Invalid format: $board\n";
	exit 1;
}

if($board =~ m/^97455/) {
	$chip = "7401";
} elsif($board =~ m/^97456/) {
	$chip = "7400";
} elsif($board =~ m/^97458/) {
	$chip = "7403";
} elsif($board =~ m/^97459/) {
	$chip = "7405";
}

if($chip eq "7413") {
	$chip = "7405";
	$rev = "d0";
} elsif($chip eq "7466" || $chip eq "7205" || $chip eq "7206") {
	$chip = "7405";
	$rev = "b0";
} elsif($chip eq "7406") {
	$chip = "7405";
} elsif($chip eq "7324") {
	$chip = "7325";
} elsif($chip eq "7402" || $chip eq "7451") {
	$chip = "7401";
} elsif($chip eq "7404" || $chip eq "7452") {
	$chip = "7403";
} elsif($chip eq "7410") {
	$chip = "7420";
} elsif($chip eq "7119" || $chip eq "7019" || $chip eq "7116" ||
		$chip eq "7117") {
	$chip = "7125";
}

print "${chip}${rev}\n";

exit 0;
