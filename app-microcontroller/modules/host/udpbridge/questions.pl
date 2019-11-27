#!/usr/bin/perl -w

use strict;
use warnings;

# Author : Nathael Pajani
# Copyright 2015 Nathael Pajani <nathael.pajani@techno-innov.fr>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


use IO::Socket::INET;
use Scalar::Util qw(looks_like_number);


#######################################
# Base package parts
my $server = 'localhost';


# UDP Helper
sub send_to {
	my ($port, $request, $type) = @_;
	my $socket = new IO::Socket::INET(Proto => 'udp');
	my $servaddr = sockaddr_in($port, inet_aton($server));
	$socket->send($request, 0, $servaddr);
	unless ($request =~ /G/) {
		print "Request sent.\n";
		return;
	}
	my $data;
	$socket->recv($data, 1024, 0);
	print "Reply: $data\n";
	$socket->close();
}


#######################################
my ($port, $req, $type, @args) = @ARGV;

unless (defined($port) && looks_like_number($port) && defined($req) && defined($type)) {
	print "Usage: $0 port request type [red green blue] ...\n";
	exit 1;
}

my $full = "$req$type ";
my $sep = ' ';
foreach (@args) {
	$full .= "$sep";
	$full .= "$_";
	#$sep = ', ';
}
print "Req to $port : $full\n";

send_to($port, $full, $type);

