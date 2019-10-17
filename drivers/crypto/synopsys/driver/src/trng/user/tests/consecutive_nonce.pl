#!/usr/bin/env perl

=head1 SYNOPSIS

consecutive_nonce.pl [-d device]

Check that the nonce handling is done properly by ensuring that the first
output of the engine after reseeds with the same nonce value are the same.

=cut

use strict;

use TRNG3::Testlib;

TRNG3::Testlib::init();
TRNG3::Testlib::sysfs_open();

my $seed = TRNG3::Testlib::open_attribute("seed", ">");
my $reseed_action = TRNG3::Testlib::open_attribute("reseed_action", ">");
my $random = TRNG3::Testlib::open_attribute("random", "<");

my $seedval = "5555555555555555555555555555555555555555555555555555555555555555";
my ($first, $second);
TRNG3::Testlib::write_attribute($seed, $seedval);
TRNG3::Testlib::write_attribute($reseed_action, "nonce");
$first = TRNG3::Testlib::read_attribute($random);

TRNG3::Testlib::write_attribute($seed, $seedval);
TRNG3::Testlib::write_attribute($reseed_action, "nonce");
$second = TRNG3::Testlib::read_attribute($random);

if ($first != $second) {
   TRNG3::Testlib::fail("consecutive nonce reseeds output different results\n"
                       ."first result:  $first\n"
                       ."second result: $second\n");
}

TRNG3::Testlib::pass();
