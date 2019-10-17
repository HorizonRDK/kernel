#!/usr/bin/env perl

=head1 SYNOPSIS

nonce_vector.pl [-d device] [-f vector]

Check that the first word of output after a nonce reseed matches the expected
value.  The first line of the vector contains the seed, the second line
contains the expected output.

=head1 OPTIONS

=over 8

=item B<-f>, B<--file>=F<file>

Specify the vector file to read (default: read from standard input).

=back

=cut

use strict;

use TRNG3::Testlib;
use IO::File;

my ($vectorfile, $vector);
TRNG3::Testlib::init('f|file=s' => \$vectorfile);

$vector = new IO::File;
if (defined $vectorfile) {
   $vector->open($vectorfile, "<")
      or die "failed to open vector $vectorfile: $!\n";
} else {
   $vector->fdopen(fileno(STDIN), "<")
      or die "failed to open stdin: $!\n";
}

TRNG3::Testlib::sysfs_open();

my $seed = TRNG3::Testlib::open_attribute("seed", ">");
my $reseed_action = TRNG3::Testlib::open_attribute("reseed_action", ">");
my $output_len = TRNG3::Testlib::open_attribute("output_len", ">");
my $random = TRNG3::Testlib::open_attribute("random", "<");

# Read vector input
my $seedval = <$vector> or die "failed to read vector: $!\n";
my $outval = <$vector> or die "failed to read vector: $!\n";

# The conversion of seed material and random output from a sequence of
# bytes to H/W register writes is not specified as this material is supposed
# to be random.  As currently implemented, the driver simply reinterprets
# the byte sequence as a sequence of 32-bit values, so the results will
# depend on host endian.  Thus, we need to run the vector twice (once in
# big endian and once in little endian) and the test will pass if either
# ordering passes.

my $seedval_be = pack "N*", reverse unpack "N*", pack "H*", $seedval;
my $seedval_le = pack "V*", reverse unpack "N*", pack "H*", $seedval;
my $outval_be = pack "N*", reverse unpack "N*", pack "H*", $outval;
my $outval_le = pack "V*", reverse unpack "N*", pack "H*", $outval;

TRNG3::Testlib::write_attribute($output_len, length $outval_be);

TRNG3::Testlib::write_attribute($seed, $seedval_be);
TRNG3::Testlib::write_attribute($reseed_action, "nonce");
my $first_be = pack "H*", TRNG3::Testlib::read_attribute($random);
TRNG3::Testlib::pass() if ($first_be eq $outval_be);

TRNG3::Testlib::write_attribute($seed, $seedval_le);
TRNG3::Testlib::write_attribute($reseed_action, "nonce");
my $first_le = pack "H*", TRNG3::Testlib::read_attribute($random);
TRNG3::Testlib::pass() if ($first_le eq $outval_le);

TRNG3::Testlib::fail("expected output does not match\n"
                    ."big endian seed:    " . unpack("H*", $seedval_be) . "\n"
                    ."expected output:    " . unpack("H*", $outval_be)  . "\n"
                    ."actual output:      " . unpack("H*", $first_be)   . "\n"
                    ."little endian seed: " . unpack("H*", $seedval_le) . "\n"
                    ."expected output:    " . unpack("H*", $outval_le)  . "\n"
                    ."actual output:      " . unpack("H*", $first_le)   . "\n"
                    );
