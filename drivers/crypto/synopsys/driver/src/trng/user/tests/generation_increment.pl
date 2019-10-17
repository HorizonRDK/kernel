#!/usr/bin/env perl

=head1 SYNOPSIS

generation_increment.pl [-d device]

Tests that seed_generation properly increments by 1 when performing a random
reseed.

=cut

use strict;

use TRNG3::Testlib;

TRNG3::Testlib::init();
TRNG3::Testlib::sysfs_open();

my $reseed_action = TRNG3::Testlib::open_attribute("reseed_action", ">");
my $seed_generation = TRNG3::Testlib::open_attribute("seed_generation", "<");

my ($start_gen, $end_gen);
$start_gen = TRNG3::Testlib::read_attribute($seed_generation);
TRNG3::Testlib::write_attribute($reseed_action, "random");
$end_gen = TRNG3::Testlib::read_attribute($seed_generation);

if ($start_gen + 1 != $end_gen) {
   TRNG3::Testlib::fail("seed_generation did not properly increment\n"
                       ."initial_generation: $start_gen\n"
                       ."final_generation:   $end_gen\n");
}

TRNG3::Testlib::pass();
