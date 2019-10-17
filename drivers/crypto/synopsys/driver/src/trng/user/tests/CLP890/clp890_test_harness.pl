#!/usr/bin/perl
# ------------------------------------------------------------------------
#
#                (C) COPYRIGHT 2012 - 2016 SYNOPSYS, INC.
#                          ALL RIGHTS RESERVED
#
#  (C) COPYRIGHT 2012-2016 Synopsys, Inc. 
#  This Synopsys software and all associated documentation are 
#  proprietary to Synopsys, Inc. and may only be used pursuant 
#  to the terms and conditions of a written license agreement 
#  with Synopsys, Inc. All other use, reproduction, modification, 
#  or distribution of the Synopsys software or the associated 
#  documentation is strictly prohibited.
#
# ------------------------------------------------------------------------

=head1 SYNOPSIS

clp890_test_harness.pl [-d device]

This is a wrapper that calls all other test scripts

=cut

use warnings;
use strict;

my $dflt_ps;
while (my $arg = shift(@ARGV)) {
  if ($arg eq "-ps") {
     $dflt_ps = shift(@ARGV);
  } elsif ($arg eq "-h") {
   print "     -ps          Specify the default personalization string.\n";
   print "     -h           Shows this help menu.\n";
  } else {
     die "invalid option ($arg). User -h for help.\n";
  }
}

# first 3 tests have random number inside. run mutiple time to cover different values.
for (my $i = 0; $i < 20; $i++) {
   if (system("./clp890_up_n_running.pl")) { print STDOUT "Up-and-Running test failed\n"; exit 1; }
   if (system("./clp890_reminders.pl"))    { print STDOUT "Reminders test failed\n";      exit 1; }
   if (system("./clp890_constraints.pl"))  { print STDOUT "Constraints test failed\n";    exit 1; }
   if (system("./clp890_other_tests.pl"))  { print STDOUT "Other test failed\n";          exit 1; }
}
if (system("./clp890_nist_vectors.pl -ps $dflt_ps")) { print STDOUT "NIST-Vectors test failed\n";   exit 1; }

exit 0;

